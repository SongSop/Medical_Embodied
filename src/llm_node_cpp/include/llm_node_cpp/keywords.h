#ifndef KEY_WORDS_H
#define KEY_WORDS_H

#include <functional>
#include <string>
#include "sherpa-onnx/c-api/c-api.h"

// 关键词检测回调函数类型
using KeywordDetectedCallback = std::function<void(const std::string& keyword)>;

class KeywordSpotter {
public:
    // 构造函数，传入配置文件路径
    KeywordSpotter(const std::string &model_path, const std::string &keywords_file);
    
    // 析构函数，释放资源
    ~KeywordSpotter();

    // 设置回调函数，当检测到关键词时调用
    void SetKeywordDetectedCallback(KeywordDetectedCallback callback);

    // 检测函数，传入采样率、音频数据和数据长度
    void DetectKeyword(int sample_rate, const float* samples, size_t num_samples);

private:
    // 配置KWS模型
    const SherpaOnnxKeywordSpotter *kws_ = nullptr;
    const SherpaOnnxOnlineStream *stream_ = nullptr;

    // 回调函数
    KeywordDetectedCallback keyword_detected_callback_;

    std::string encoder_path;
    std::string decoder_path;
    std::string joiner_path;
    std::string tokens_path;
};



#endif // KEY_WORDS_H
