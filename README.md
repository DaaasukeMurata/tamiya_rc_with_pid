![All System image](https://raw.githubusercontent.com/DaaasukeMurata/rc_w_pid/images/system.jpg)

結果 : 上手に走った。

- [走行Movie1](https://drive.google.com/open?id=0B0mNEspU9cAiX093OWIxSTVVcDg)
- [走行Movie2](https://drive.google.com/open?id=0B0mNEspU9cAidE1nUGxNbDBiMGM)
- [走行Movie3](https://drive.google.com/open?id=0B0mNEspU9cAiT0c3R3pBVHZsMnM)


# index

<!-- TOC -->

- [index](#index)
- [動作環境](#動作環境)
- [実行](#実行)
    - [白線位置検出](#白線位置検出)
    - [ラジコン制御](#ラジコン制御)

<!-- /TOC -->


# 動作環境

- OpenCV 2.4.8
- ROS Indigo
- Python 2.7

# 実行

```
$ roslaunch rc_pid rc_pid.launch
```

## 白線位置検出

カメラ映像を入力として受け取り

![CAM img](https://raw.githubusercontent.com/DaaasukeMurata/rc_w_pid/images/cam_original.jpg)

映像の一部をトリミング

![CAM img](https://raw.githubusercontent.com/DaaasukeMurata/rc_w_pid/images/cam_process1.jpg)

mono color化、ブラー、edge検出、輪郭抽出で、画像を2値化

![CAM img](https://raw.githubusercontent.com/DaaasukeMurata/rc_w_pid/images/cam_process2.jpg)

白の箇所を白線と認識。端に映る壁を誤検知しないよう、まず中心付近で探し、なければ端も探す。

![CAM img](https://raw.githubusercontent.com/DaaasukeMurata/rc_w_pid/images/cam_process3.jpg)

## ラジコン制御

- 白線が中心に来るよう、ステアリングをPID制御。[（PID解説記事 - Qiita）](http://qiita.com/RyosukeH/items/9e5ce2ebdadd90e3db00)

- Kp, Ki, Kdのパラメータは限界感度法で調整。[（解説記事）](http://monoist.atmarkit.co.jp/mn/articles/1007/26/news083.html)

