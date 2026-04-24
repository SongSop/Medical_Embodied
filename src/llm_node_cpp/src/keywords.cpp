#include "llm_node_cpp/keywords.h"
#include <stdio.h>
#include <stdlib.h> // exit
#include <string.h> // memset

#include <cstring>

#include "llm_node_cpp/conf.h"

// int32_t main()
// {
//     // 配置 KWS 模型
//     SherpaOnnxKeywordSpotterConfig config;

//     memset(&config, 0, sizeof(config));
//     config.model_config.transducer.encoder =
//         "/opt/sherpa-onnx/kws-zipformer-wenetspeech/"
//         "encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx";

//     config.model_config.transducer.decoder =
//         "/opt/sherpa-onnx/kws-zipformer-wenetspeech/"
//         "decoder-epoch-12-avg-2-chunk-16-left-64.onnx";

//     config.model_config.transducer.joiner =
//         "/opt/sherpa-onnx/kws-zipformer-wenetspeech/"
//         "joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx";

//     config.model_config.tokens =
//         "/opt/sherpa-onnx/kws-zipformer-wenetspeech/"
//         "tokens.txt";

//     config.model_config.provider = "cpu";
//     config.model_config.num_threads = 6;
//     config.model_config.debug = 1;

//     // keywords file
//     config.keywords_file =
//         "./sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01-mobile/"
//         "test_wavs/test_keywords.txt";

//     // 从关键词文件中创建 KWS 模型
//     const SherpaOnnxKeywordSpotter *kws = SherpaOnnxCreateKeywordSpotter(&config);
//     if (!kws) {
//         err_log("createing kws failed.");
//     }

//     // const char *wav_filename =
//     //     "./sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01-mobile/"
//     //     "test_wavs/3.wav";

//     // const SherpaOnnxWave *wave = SherpaOnnxReadWave(wav_filename);
//     // if (wave == NULL)
//     // {
//     //     fprintf(stderr, "Failed to read %s\n", wav_filename);
//     //     exit(-1);
//     // }

//     // 创建 KWS stream
//     const SherpaOnnxOnlineStream *stream = SherpaOnnxCreateKeywordStream(kws);
//     if (!stream) {
//         err_log("Failed to create stream");
//     }

//     // 输入音频数据 进行检测, 参数分别是 stream, 采样率, 采样数据地址, 采样数据的长度
//     SherpaOnnxOnlineStreamAcceptWaveform(stream, wave->sample_rate, wave->samples,
//         wave->num_samples);

//     // 静音补充数组，用来 确保音频完整，防止截断检测不准确
//     static float tail_paddings[8000] = {0}; // 0.5 seconds
//     SherpaOnnxOnlineStreamAcceptWaveform(stream, wave->sample_rate, tail_paddings,
//             sizeof(tail_paddings) / sizeof(float));

//     // 标记音频输入结束
//     SherpaOnnxOnlineStreamInputFinished(stream);

//     while (SherpaOnnxIsKeywordStreamReady(kws, stream))
//     {
//         SherpaOnnxDecodeKeywordStream(kws, stream);
//         const SherpaOnnxKeywordResult *r = SherpaOnnxGetKeywordResult(kws, stream);
//         if (r && r->json && strlen(r->keyword))
//         {
//             fprintf(stderr, "Detected keyword: %s\n", r->json);

//             // Remember to reset the keyword stream
//             SherpaOnnxResetKeywordStream(kws, stream);
//         }
//         SherpaOnnxDestroyKeywordResult(r);
//     }
//     SherpaOnnxDestroyOnlineStream(stream);

//     return 0;
// }


KeywordSpotter::KeywordSpotter(const std::string &model_path, const std::string &keywords_file) {
    sys_log("model_path: %s", model_path.c_str());
    sys_log("keywords_file: %s", keywords_file.c_str());

    // 拼接路径并存储为成员变量
    encoder_path = model_path + "/encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx";
    decoder_path = model_path + "/decoder-epoch-12-avg-2-chunk-16-left-64.onnx";
    joiner_path = model_path + "/joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx";
    tokens_path = model_path + "/tokens.txt";

    // 配置 KWS 模型
    SherpaOnnxKeywordSpotterConfig config;
    memset(&config, 0, sizeof(config));
    config.model_config.transducer.encoder = encoder_path.c_str();
    config.model_config.transducer.decoder = decoder_path.c_str();
    config.model_config.transducer.joiner = joiner_path.c_str();
    config.model_config.tokens = tokens_path.c_str();
    config.model_config.provider = "cpu";
    config.model_config.num_threads = 6;
    config.model_config.debug = 0;
    config.keywords_file = keywords_file.c_str();

    // 从关键词文件中创建 KWS 模型
    kws_ = SherpaOnnxCreateKeywordSpotter(&config);
    if (!kws_) {
        err_log("Creating kws failed!");
        return;
    }

    // 创建 KWS stream
    stream_ = SherpaOnnxCreateKeywordStream(kws_);  
    if (!stream_) {
        err_log("Failed to create stream");
        return;
    }
}

KeywordSpotter::~KeywordSpotter() {

    sys_log("destory sherpa-onnx keywords stream...");
    if (stream_) {
        SherpaOnnxDestroyOnlineStream(stream_);
        stream_ = nullptr;
    }

    sys_log("destory sherpa-onnx keywords model...");
    if (kws_) {
        SherpaOnnxDestroyKeywordSpotter(kws_);
        kws_ = nullptr;
    }
}


void KeywordSpotter::SetKeywordDetectedCallback(KeywordDetectedCallback callback) {
    keyword_detected_callback_ = callback;
}

void KeywordSpotter::DetectKeyword(int sample_rate, const float* samples, size_t num_samples) {
    if (!kws_) {
        err_log("KWS model not initialized!");
        return;
    }

    // 输入音频数据 进行检测
    SherpaOnnxOnlineStreamAcceptWaveform(stream_, sample_rate, samples, num_samples);

    // 静音补充数组
    // static float tail_paddings[8000] = {0}; // 0.5 seconds
    // SherpaOnnxOnlineStreamAcceptWaveform(stream_, sample_rate, tail_paddings, sizeof(tail_paddings) / sizeof(float));

    // 标记音频输入结束
    SherpaOnnxOnlineStreamInputFinished(stream_);

    while (SherpaOnnxIsKeywordStreamReady(kws_, stream_)) {  // 检查是否准备好
        // 检测关键词
        SherpaOnnxDecodeKeywordStream(kws_, stream_);
        // 获取检测结果
        const SherpaOnnxKeywordResult *r = SherpaOnnxGetKeywordResult(kws_, stream_);
        
        if (r && r->json && strlen(r->keyword)) {
            // 输出检测到的关键词
            std::string detected_keyword = r->json;
            if (keyword_detected_callback_) {
                keyword_detected_callback_(detected_keyword);
            }

            // 重置关键词流
            SherpaOnnxResetKeywordStream(kws_, stream_);

            // 保险起见，可以尝试重新创建关键词流
            if (stream_) {
                stream_ = SherpaOnnxCreateKeywordStream(kws_);  
                if (!stream_) {
                    err_log("Failed to create stream");
                    return;
                }
            }
        }
        SherpaOnnxDestroyKeywordResult(r);
    }
}
