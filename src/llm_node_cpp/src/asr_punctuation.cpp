#include "llm_node_cpp/asr_punctuation.h"
#include <iostream>
#include <cstring>

#define  ASR_PUNCTUATION_DEBUG_ENBAL    1
#define  ASR_PUNCTUATION_NUM_THREADS    6

AsrPunctuation::AsrPunctuation(const std::string& model_path) {

    SherpaOnnxOfflinePunctuationConfig config;
    std::memset(&config, 0, sizeof(config));

    config.model.ct_transformer = model_path.c_str();
    config.model.num_threads = ASR_PUNCTUATION_NUM_THREADS;
    config.model.debug = ASR_PUNCTUATION_DEBUG_ENBAL;
    config.model.provider = "cpu";

    punct_ = (SherpaOnnxOfflinePunctuation*)SherpaOnnxCreateOfflinePunctuation(&config);
    if (!punct_) {
        throw std::runtime_error("Failed to create OfflinePunctuation. Please check your model path.");
    }
}

AsrPunctuation::~AsrPunctuation() {
    if (punct_) {
        SherpaOnnxDestroyOfflinePunctuation(punct_);
    }
}

std::string AsrPunctuation::AddPunctuation(const std::string& text) {
    if (!punct_) {
        throw std::runtime_error("Punctuation model is not initialized.");
    }

    const char* text_with_punct = SherpaOfflinePunctuationAddPunct(punct_, text.c_str());
    if (!text_with_punct) {
        throw std::runtime_error("Failed to process text with punctuation.");
    }

    std::string result(text_with_punct);
    SherpaOfflinePunctuationFreeText(text_with_punct);
    return result;
}
