#ifndef ASR_PUNCTUATION_H
#define ASR_PUNCTUATION_H

#include <string>
#include "sherpa-onnx/c-api/c-api.h"

// 这个类用于给识别出的文本添加标点
class AsrPunctuation {
public:
    explicit AsrPunctuation(const std::string& model_path);
    ~AsrPunctuation();

    // 传入要添加标点的 string ，返回已经添加标点的 string 
    std::string AddPunctuation(const std::string& text);

private:
    SherpaOnnxOfflinePunctuation* punct_;
};

#endif // ASR_PUNCTUATION_H
