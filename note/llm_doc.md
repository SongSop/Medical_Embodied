




在工控机上测试了

python test_hci_action.py --mode 1

python test_hci_action.py --mode passive

测试记录：
```bash

(qwen_tts_online) ➜  dialog git:(dev) ✗ python test_hci_action.py --mode 1
[INFO] [1778402672.931608230] [test_hci_action_client]: send goal: mode=1, person_id=1, context=none
[INFO] [1778402680.674863550] [test_hci_action_client]: feedback: Received context for person 1
[INFO] [1778402680.675434699] [test_hci_action_client]: feedback: running
[INFO] [1778402680.877085505] [test_hci_action_client]: feedback: running
[INFO] [1778402681.079240095] [test_hci_action_client]: feedback: running
[INFO] [1778402681.279699559] [test_hci_action_client]: feedback: running
[INFO] [1778402681.479388114] [test_hci_action_client]: feedback: running
[INFO] [1778402681.683477459] [test_hci_action_client]: feedback: running
[INFO] [1778402681.882986293] [test_hci_action_client]: feedback: running
[INFO] [1778402682.082653091] [test_hci_action_client]: feedback: running
[INFO] [1778402682.284249411] [test_hci_action_client]: feedback: running
[INFO] [1778402682.483742455] [test_hci_action_client]: feedback: running
[INFO] [1778402682.685293363] [test_hci_action_client]: feedback: running
[INFO] [1778402682.886536972] [test_hci_action_client]: feedback: running
[INFO] [1778402683.086382684] [test_hci_action_client]: feedback: running
[INFO] [1778402683.288375463] [test_hci_action_client]: feedback: running
[INFO] [1778402683.489970742] [test_hci_action_client]: feedback: running
[INFO] [1778402683.689397952] [test_hci_action_client]: feedback: running
[INFO] [1778402683.891235759] [test_hci_action_client]: feedback: running
[INFO] [1778402684.090813518] [test_hci_action_client]: feedback: running
[INFO] [1778402684.290576762] [test_hci_action_client]: feedback: running
[INFO] [1778402684.492087514] [test_hci_action_client]: feedback: running
[INFO] [1778402684.693416364] [test_hci_action_client]: feedback: running
[INFO] [1778402684.892862794] [test_hci_action_client]: feedback: running
[INFO] [1778402685.092916278] [test_hci_action_client]: feedback: running
[INFO] [1778402685.294403755] [test_hci_action_client]: feedback: running
[INFO] [1778402685.495082824] [test_hci_action_client]: feedback: running
[INFO] [1778402685.695267131] [test_hci_action_client]: feedback: running
[INFO] [1778402685.895427043] [test_hci_action_client]: feedback: running
[INFO] [1778402686.095334970] [test_hci_action_client]: action result: status=0, need_call_nurse=True, summary=患者发出呼叫护士的请求（“帮我小护士”），需要立即通知护士。
(qwen_tts_online) ➜  dialog git:(dev) ✗ python test_hci_action.py --mode passive
[INFO] [1778402808.160999637] [test_hci_action_client]: send goal: mode=1, person_id=1, context=none
[INFO] [1778402814.749241034] [test_hci_action_client]: feedback: Received context for person 1
[INFO] [1778402814.749611401] [test_hci_action_client]: feedback: running
[INFO] [1778402814.951303386] [test_hci_action_client]: feedback: running
[INFO] [1778402815.150930051] [test_hci_action_client]: feedback: running
[INFO] [1778402815.354835027] [test_hci_action_client]: feedback: running
[INFO] [1778402815.553180674] [test_hci_action_client]: feedback: running
[INFO] [1778402815.757323929] [test_hci_action_client]: feedback: running
[INFO] [1778402815.956991149] [test_hci_action_client]: feedback: running
[INFO] [1778402816.156641262] [test_hci_action_client]: feedback: running
[INFO] [1778402816.357972470] [test_hci_action_client]: feedback: running
[INFO] [1778402816.558645123] [test_hci_action_client]: feedback: running
[INFO] [1778402816.762320403] [test_hci_action_client]: feedback: running
[INFO] [1778402816.960155916] [test_hci_action_client]: feedback: running
[INFO] [1778402817.160664398] [test_hci_action_client]: feedback: running
[INFO] [1778402817.361093312] [test_hci_action_client]: feedback: running
[INFO] [1778402817.561611745] [test_hci_action_client]: feedback: running
[INFO] [1778402817.760986015] [test_hci_action_client]: feedback: running
[INFO] [1778402817.962923731] [test_hci_action_client]: feedback: running
[INFO] [1778402818.163005246] [test_hci_action_client]: feedback: running
[INFO] [1778402818.364217463] [test_hci_action_client]: feedback: running
[INFO] [1778402818.563966914] [test_hci_action_client]: feedback: running
[INFO] [1778402818.766661213] [test_hci_action_client]: feedback: running
[INFO] [1778402818.967455568] [test_hci_action_client]: feedback: running
[INFO] [1778402819.168293899] [test_hci_action_client]: feedback: running
[INFO] [1778402819.368533692] [test_hci_action_client]: feedback: running
[INFO] [1778402819.568739159] [test_hci_action_client]: action result: status=0, need_call_nurse=True, summary=患者明确要求呼叫护士，需要立即通知医护人员。
(qwen_tts_online) ➜  dialog git:(dev) ✗

```




```bash
(qwen_tts_online) ➜  dialog git:(dev) ✗ python test_hci_action.py --mode interrupt
[INFO] [1778402894.014254449] [test_hci_action_client]: send goal: mode=2, person_id=1, context=none
[INFO] [1778402900.211550518] [test_hci_action_client]: feedback: Received context for person 1
[INFO] [1778402900.212446092] [test_hci_action_client]: feedback: running
[INFO] [1778402900.413292540] [test_hci_action_client]: feedback: running
[INFO] [1778402900.613291342] [test_hci_action_client]: feedback: running
[INFO] [1778402900.813795153] [test_hci_action_client]: feedback: running
[INFO] [1778402901.014710382] [test_hci_action_client]: feedback: running
[INFO] [1778402901.215006085] [test_hci_action_client]: feedback: running
[INFO] [1778402901.415839705] [test_hci_action_client]: feedback: running
[INFO] [1778402901.615034964] [test_hci_action_client]: feedback: running
[INFO] [1778402901.817709338] [test_hci_action_client]: feedback: running
[INFO] [1778402902.016305161] [test_hci_action_client]: feedback: running
[INFO] [1778402902.218204517] [test_hci_action_client]: feedback: running
[INFO] [1778402902.418100157] [test_hci_action_client]: feedback: running
[INFO] [1778402902.619476106] [test_hci_action_client]: feedback: running
[INFO] [1778402902.821148263] [test_hci_action_client]: feedback: running
[INFO] [1778402903.021361611] [test_hci_action_client]: feedback: running
[INFO] [1778402903.223360791] [test_hci_action_client]: feedback: running
[INFO] [1778402903.424347570] [test_hci_action_client]: feedback: running
[INFO] [1778402903.626663305] [test_hci_action_client]: feedback: running
[INFO] [1778402903.826020906] [test_hci_action_client]: feedback: running
[INFO] [1778402904.026401331] [test_hci_action_client]: feedback: running
[INFO] [1778402904.226799604] [test_hci_action_client]: feedback: running
[INFO] [1778402904.427831334] [test_hci_action_client]: feedback: running
[INFO] [1778402904.627463658] [test_hci_action_client]: feedback: running
[INFO] [1778402904.828472572] [test_hci_action_client]: feedback: running
[INFO] [1778402905.027534621] [test_hci_action_client]: feedback: running
[INFO] [1778402905.229191431] [test_hci_action_client]: feedback: running
[INFO] [1778402905.430258261] [test_hci_action_client]: feedback: running
[INFO] [1778402905.630560176] [test_hci_action_client]: feedback: running
[INFO] [1778402905.830033577] [test_hci_action_client]: action result: status=0, need_call_nurse=True, summary=患者明确要求呼叫护士，无论具体称呼如何，都视为需要医护人员介入的请求。
```




```bash
(qwen_tts_online) ➜  dialog git:(dev) ✗ python test_hci_action.py --mode alert
[INFO] [1778402948.545843562] [test_hci_action_client]: send goal: mode=0, person_id=1, context=none
[INFO] [1778402958.161842247] [test_hci_action_client]: action result: status=0, need_call_nurse=False, summary=alert patient, no need for calling nurse.
```




