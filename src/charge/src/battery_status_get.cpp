#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <interfaces/msg/battery.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/bool.hpp>

/*
 * 串口BMS读取节点（循环收发）
 *
 * 发送（固定）: dd a5 03 00 ff fd 77
 *
 * 接收帧格式：
 * DD | CMD | STATUS | LEN | DATA(LEN字节) | CRC_H | CRC_L | 77
 *
 * 校验：从第2字节(STATUS)到校验码前1字节求和，取反+1，取低16位。
 *
 * 数据字段（从第0字节开始）：
 * 4-5   : 电压(除100得到V)
 * 6-7   : 电流（最高位=1放电；放电电流=-(0xFFFF-raw)，否则为充电电流）
 * 8-9   : 剩余容量(10mAh)
 * 10-11 : 总容量(10mAh)
 * 12-13 : 充电次数
 * 14-15 : 生产日期(raw)
 * 22    : 软件版本
 * 23    : 容量百分比SOC(%)
 * 24    : MOS状态
 * 25    : 电池串数
 * 26    : 温度探头个数
 * 27-28 : 温度1(raw -> (raw-2731)/10 ℃)
 * 29-30 : 温度2
 * 31-32 : 温度3
 */

namespace
{
uint16_t u16be(const std::vector<uint8_t>& v, size_t idx)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(v[idx]) << 8) | static_cast<uint16_t>(v[idx + 1]));
}

float tempCFromRaw(uint16_t raw)
{
    return (static_cast<int>(raw) - 2731) / 10.0f;
}

std::string sanitizeForFilename(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (unsigned char ch : s)
    {
        if (std::isalnum(ch))
            out.push_back(static_cast<char>(ch));
        else
            out.push_back('_');
    }
    return out;
}

std::vector<uint8_t> parseHexBytes(const std::string& s)
{
    std::vector<uint8_t> out;
    int hi = -1;
    for (unsigned char ch : s)
    {
        if (std::isspace(ch) || ch == ',' || ch == ';')
            continue;
        int v = -1;
        if (ch >= '0' && ch <= '9')
            v = ch - '0';
        else if (ch >= 'a' && ch <= 'f')
            v = ch - 'a' + 10;
        else if (ch >= 'A' && ch <= 'F')
            v = ch - 'A' + 10;
        else
            continue;

        if (hi < 0)
            hi = v;
        else
        {
            out.push_back(static_cast<uint8_t>((hi << 4) | v));
            hi = -1;
        }
    }
    return out;
}
}  // namespace

class BatterySerialNode : public rclcpp::Node
{
public:
    struct BatteryStatus
    {
        std::vector<uint8_t> raw_frame;
        uint16_t crc_calc = 0;
        uint16_t crc_recv = 0;

        uint8_t cmd = 0;
        uint8_t status = 0;
        uint8_t len = 0;

        float voltage_v = std::numeric_limits<float>::quiet_NaN();
        uint16_t current_raw = 0;
        int current_ma = 0;  // 放电为负，充电为正（按协议原始单位）
        bool charging = false;

        uint16_t remain_cap_10mah = 0;
        uint16_t total_cap_10mah = 0;
        uint16_t cycle_count = 0;
        uint16_t prod_date_raw = 0;

        uint8_t sw_version = 0;
        float soc_percent = std::numeric_limits<float>::quiet_NaN();
        uint8_t mos_status = 0;
        uint8_t series_count = 0;
        uint8_t temp_probe_count = 0;
        std::array<float, 3> temps_c{{std::numeric_limits<float>::quiet_NaN(),
                                      std::numeric_limits<float>::quiet_NaN(),
                                      std::numeric_limits<float>::quiet_NaN()}};
    };

    explicit BatterySerialNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node("battery_status_get", options)
    {
        port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baudrate_ = declare_parameter<int>("baudrate", 9600);
        query_hz_ = declare_parameter<double>("query_hz", 0.5);
        response_timeout_ms_ = declare_parameter<int>("response_timeout_ms", 800);
        max_consecutive_failures_ = declare_parameter<int>("max_consecutive_failures", 3);
        battery_topic_ = declare_parameter<std::string>("battery_topic", "/battery");
        debug_hex_ = declare_parameter<bool>("debug_hex", false);

        write_trigger_topic_ = declare_parameter<std::string>("write_trigger_topic", "/battery/serial_write_trigger");
        reserved_write_hex_ = declare_parameter<std::string>("reserved_write_hex", std::string());
        reserved_write_bytes_ = parseHexBytes(reserved_write_hex_);

        openPortLockOrThrow();

        pub_ = create_publisher<interfaces::msg::Battery>(battery_topic_, 10);
        write_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
            write_trigger_topic_,
            10,
            std::bind(&BatterySerialNode::onWriteTrigger, this, std::placeholders::_1));

        request_cmd_ = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};

        openSerial();
    }

    ~BatterySerialNode()
    {
        try
        {
            if (ser_.isOpen())
                ser_.close();
        }
        catch (...)
        {
        }

        if (lock_fd_ >= 0)
        {
            ::close(lock_fd_);
            lock_fd_ = -1;
        }
    }

    void spin()
    {
        if (query_hz_ <= 0.0)
            query_hz_ = 0.5;

        const auto query_period = std::chrono::duration<double>(1.0 / query_hz_);
        const auto loop_sleep = std::chrono::milliseconds(20);

        auto last_query = std::chrono::steady_clock::now();
        bool first_query = true;
        int consecutive_failures = 0;

        while (rclcpp::ok())
        {
            rclcpp::spin_some(shared_from_this());

            // 预留写：topic=true 时触发写串口（数据预留）
            if (pending_reserved_write_.exchange(false))
            {
                if (!reserved_write_bytes_.empty())
                {
                    ensureSerialOpen();
                    safeWrite(reserved_write_bytes_);
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000, "Reserved write triggered, but reserved_write_hex is empty.");
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (first_query || (now - last_query) >= query_period)
            {
                first_query = false;
                last_query = now;

                if (!ensureSerialOpen())
                {
                    std::this_thread::sleep_for(loop_sleep);
                    continue;
                }

                BatteryStatus st;
                std::string err;
                const bool ok = queryOnce(st, err);

                if (!ok)
                {
                    ++consecutive_failures;
                    RCLCPP_WARN_THROTTLE(get_logger(),
                                         *get_clock(),
                                         2000,
                                         "Battery query failed (%d/%d): %s",
                                         consecutive_failures,
                                         max_consecutive_failures_,
                                         err.c_str());

                    if (consecutive_failures >= max_consecutive_failures_)
                    {
                        RCLCPP_ERROR(get_logger(),
                                     "No valid response for %d consecutive cycles, restarting serial...",
                                     consecutive_failures);
                        restartSerial();
                        consecutive_failures = 0;
                    }
                }
                else
                {
                    consecutive_failures = 0;

                    interfaces::msg::Battery msg;
                    msg.soc = st.soc_percent;
                    msg.charging = st.charging;
                    msg.voltage = st.voltage_v;
                    pub_->publish(msg);

                    last_status_ = st;  // 额外数据保留，后续可扩展处理
                }
            }

            std::this_thread::sleep_for(loop_sleep);
        }
    }

private:
    bool ensureSerialOpen()
    {
        if (ser_.isOpen())
            return true;
        return openSerial();
    }

    bool openSerial()
    {
        try
        {
            if (ser_.isOpen())
                ser_.close();

            ser_.setPort(port_);
            ser_.setBaudrate(static_cast<uint32_t>(baudrate_));
            // 用较大的读超时，避免 waitReadable/available 抖动导致读不到数据
            serial::Timeout timeout = serial::Timeout::simpleTimeout(std::max(200, response_timeout_ms_));
            ser_.setTimeout(timeout);
            ser_.setBytesize(serial::eightbits);
            ser_.setParity(serial::parity_none);
            ser_.setStopbits(serial::stopbits_one);
            ser_.setFlowcontrol(serial::flowcontrol_none);
            ser_.open();

            if (!ser_.isOpen())
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Serial open failed (unknown), port=%s", port_.c_str());
                return false;
            }

            ser_.flushInput();
            ser_.flushOutput();
            rx_buffer_.clear();

            RCLCPP_INFO(get_logger(), "Serial opened: %s @ %d", port_.c_str(), baudrate_);
            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Serial open exception: %s", e.what());
            return false;
        }
    }

    void restartSerial()
    {
        try
        {
            if (ser_.isOpen())
                ser_.close();
        }
        catch (...)
        {
        }
        rx_buffer_.clear();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        openSerial();
    }

    void safeWrite(const std::vector<uint8_t>& bytes)
    {
        if (!ser_.isOpen())
            return;
        // 清掉上一次残留（避免拼帧被污染）
        rx_buffer_.clear();
        ser_.write(bytes.data(), bytes.size());
        ser_.flush();
    }

    void drainInputNonBlocking()
    {
        if (!ser_.isOpen())
            return;
        while (ser_.available() > 0)
        {
            std::vector<uint8_t> tmp(std::min<size_t>(ser_.available(), 256));
            const size_t n = ser_.read(tmp.data(), tmp.size());
            if (n == 0)
                break;
        }
    }

    bool queryOnce(BatteryStatus& st, std::string& err)
    {
        err.clear();

        // 读走可能残留的数据，避免上一轮半帧干扰
        drainInputNonBlocking();

        // 发查询
        safeWrite(request_cmd_);

        // 读整帧
        std::vector<uint8_t> frame;
        if (!readOneFrame(frame, response_timeout_ms_))
        {
            err = "timeout/no valid frame";
            return false;
        }

        // 解析
        if (!parseFrame(frame, st, err))
            return false;

        return true;
    }

    bool readOneFrame(std::vector<uint8_t>& frame_out, int timeout_ms)
    {
        const auto start = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::milliseconds(std::max(1, timeout_ms));

        // 采用 waitReadable + read 方式，避免仅靠 available() 轮询导致一直读不到
        while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < timeout)
        {
            const bool readable = ser_.waitReadable();
            if (readable)
            {
                size_t n = ser_.available();
                if (n == 0)
                    n = 1;

                std::vector<uint8_t> temp(n);
                const size_t got = ser_.read(temp.data(), n);
                temp.resize(got);
                if (!temp.empty())
                    rx_buffer_.insert(rx_buffer_.end(), temp.begin(), temp.end());
            }

            if (extractOneFrameFromBuffer(frame_out))
            {
                if (debug_hex_)
                {
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000, "Battery frame received, len=%zu", frame_out.size());
                }
                return true;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
    }

    bool extractOneFrameFromBuffer(std::vector<uint8_t>& frame_out)
    {
        while (true)
        {
            if (rx_buffer_.empty())
                return false;

            // sync head
            if (rx_buffer_[0] != 0xDD)
            {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            if (rx_buffer_.size() < 4)
                return false;

            const uint8_t len = rx_buffer_[3];
            const size_t total_len = static_cast<size_t>(7 + len);  // 4 + LEN + 2 + 1

            if (rx_buffer_.size() < total_len)
                return false;

            if (rx_buffer_[total_len - 1] != 0x77)
            {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            std::vector<uint8_t> frame(rx_buffer_.begin(), rx_buffer_.begin() + total_len);
            if (!checkCrc(frame))
            {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            frame_out.swap(frame);
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_len);
            return true;
        }
    }

    bool checkCrc(const std::vector<uint8_t>& frame)
    {
        if (frame.size() < 7)
            return false;
        uint16_t sum = 0;
        for (size_t i = 2; i < frame.size() - 3; ++i)
            sum = static_cast<uint16_t>(sum + frame[i]);
        const uint16_t calc = static_cast<uint16_t>(((~sum) + 1) & 0xFFFF);
        const uint16_t recv = static_cast<uint16_t>((static_cast<uint16_t>(frame[frame.size() - 3]) << 8) |
                                                    static_cast<uint16_t>(frame[frame.size() - 2]));
        return calc == recv;
    }

    bool parseFrame(const std::vector<uint8_t>& frame, BatteryStatus& out, std::string& err)
    {
        if (frame.size() < 7)
        {
            err = "frame too short";
            return false;
        }
        if (frame.front() != 0xDD || frame.back() != 0x77)
        {
            err = "bad head/tail";
            return false;
        }
        if (frame[1] != 0x03)
        {
            err = "unexpected CMD (not 0x03)";
            return false;
        }

        const uint8_t len = frame[3];
        const size_t expect_total = static_cast<size_t>(7 + len);
        if (frame.size() != expect_total)
        {
            err = "LEN mismatch";
            return false;
        }

        // 需要至少解析到温度3
        if (frame.size() <= 32)
        {
            err = "frame too short for required fields";
            return false;
        }

        // 计算CRC并记录（checkCrc 已经通过）
        uint16_t sum = 0;
        for (size_t i = 2; i < frame.size() - 3; ++i)
            sum = static_cast<uint16_t>(sum + frame[i]);
        const uint16_t crc_calc = static_cast<uint16_t>(((~sum) + 1) & 0xFFFF);
        const uint16_t crc_recv = static_cast<uint16_t>((static_cast<uint16_t>(frame[frame.size() - 3]) << 8) |
                                                        static_cast<uint16_t>(frame[frame.size() - 2]));

        BatteryStatus st;
        st.raw_frame = frame;
        st.crc_calc = crc_calc;
        st.crc_recv = crc_recv;

        st.cmd = frame[1];
        st.status = frame[2];
        st.len = len;

        const uint16_t voltage_raw = u16be(frame, 4);
        st.voltage_v = voltage_raw / 100.0f;

        st.current_raw = u16be(frame, 6);
        if ((st.current_raw & 0x8000) != 0)
        {
            st.current_ma = -static_cast<int>(0xFFFFu - st.current_raw);
            st.charging = false;
        }
        else
        {
            st.current_ma = static_cast<int>(st.current_raw);
            st.charging = (st.current_ma > 0);
        }

        st.remain_cap_10mah = u16be(frame, 8);
        st.total_cap_10mah = u16be(frame, 10);
        st.cycle_count = u16be(frame, 12);
        st.prod_date_raw = u16be(frame, 14);

        st.sw_version = frame[22];
        st.soc_percent = static_cast<float>(frame[23]);
        st.mos_status = frame[24];
        st.series_count = frame[25];
        st.temp_probe_count = frame[26];
        st.temps_c[0] = tempCFromRaw(u16be(frame, 27));
        st.temps_c[1] = tempCFromRaw(u16be(frame, 29));
        st.temps_c[2] = tempCFromRaw(u16be(frame, 31));

        if (!std::isfinite(st.voltage_v) || st.voltage_v < 0.0f || st.voltage_v > 200.0f)
        {
            err = "voltage out of range";
            return false;
        }
        if (!std::isfinite(st.soc_percent) || st.soc_percent < 0.0f || st.soc_percent > 100.0f)
        {
            err = "soc out of range";
            return false;
        }

        out = st;
        return true;
    }

    void onWriteTrigger(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg && msg->data)
            pending_reserved_write_.store(true);
    }

    void openPortLockOrThrow()
    {
        const std::string lock_path = std::string("/tmp/") + "battery_serial_" + sanitizeForFilename(port_) + ".lock";
        const int fd = ::open(lock_path.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd < 0)
        {
            const int e = errno;
            throw std::runtime_error("open lock file failed: " + lock_path + " errno=" + std::to_string(e));
        }

        if (::flock(fd, LOCK_EX | LOCK_NB) != 0)
        {
            const int e = errno;
            ::close(fd);
            throw std::runtime_error("serial port locked by another process: " + lock_path + " errno=" +
                                     std::to_string(e));
        }

        lock_fd_ = fd;
    }

private:
    rclcpp::Publisher<interfaces::msg::Battery>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr write_trigger_sub_;

    serial::Serial ser_;
    std::string port_;
    int baudrate_ = 9600;

    double query_hz_ = 0.5;
    int response_timeout_ms_ = 800;
    int max_consecutive_failures_ = 3;
    std::string battery_topic_;

    std::string write_trigger_topic_;
    std::string reserved_write_hex_;
    std::vector<uint8_t> reserved_write_bytes_;
    std::atomic<bool> pending_reserved_write_{false};

    std::vector<uint8_t> request_cmd_;
    std::vector<uint8_t> rx_buffer_;

    int lock_fd_ = -1;

    BatteryStatus last_status_;

    bool debug_hex_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<BatterySerialNode>();
        node->spin();
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("battery_status_get"), "battery_status_get init failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}
