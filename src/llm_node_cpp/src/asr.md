

模型的地址：https://k2-fsa.github.io/sherpa/onnx/pretrained_models/online-transducer/zipformer-transducer-models.html#csukuangfj-sherpa-onnx-streaming-zipformer-zh-14m-2023-02-23-chinese



## 关于 endpoint

参考链接： https://k2-fsa.github.io/sherpa/ncnn/endpoint.html

有三条规则来判断 endpoint

规则1：检测到一段时间的静音，默认时间是 2.4s

规则2：在两段的语音之间有一段静音，这段静音如果超过了设定的时间，也认定为 endpoint，默认是 1.2s

规则3：语音时间过长，默认值是 20s ，超过 20s 的话也会添加一个 endpoint
