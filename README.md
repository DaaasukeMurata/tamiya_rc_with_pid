![All System image](https://raw.githubusercontent.com/DaaasukeMurata/rc_w_dl/images/all_system.jpg)

結果 : コーナーは曲がるが、直線ですぐ壁にぶつかる。[（走行Movie）](https://drive.google.com/open?id=0B0mNEspU9cAiS2N3SVBUQlBHNjQ)


# index

<!-- TOC -->

- [index](#index)
- [動作環境](#動作環境)
- [流れ](#流れ)
- [1. データ取得](#1-データ取得)
- [2. 学習用データへの加工](#2-学習用データへの加工)
    - [カメラ画像の加工パラメータ調整（ros/rc_image_w_tf/scripts/image_process/）](#カメラ画像の加工パラメータ調整rosrc_image_w_tfscriptsimage_process)
    - [学習データの作成（create_testdata）](#学習データの作成create_testdata)
- [3. TensorFlowで学習（learning/）](#3-tensorflowで学習learning)
    - [Training](#training)
    - [Result](#result)
- [4. ラジコン制御（ros/rc_image_w_tf/）](#4-ラジコン制御rosrc_image_w_tf)

<!-- /TOC -->


# 動作環境

- TensorFlow 0.11.0
- OpenCV 2.4.8
- ROS Indigo
- Python 2.7
- PyQt4

# 流れ

1. ラジコンを操作して、カメラ画像とステアリングのデータ取得

2. DeepLearning学習用のテストデータに加工

3. TensorFlowで学習

4. 学習済みモデルを使用して、ラジコンのカメラ画像からステアリング値取得 -> ラジコン制御


# 1. データ取得

カメラを取り付けたラジコンを、手動で操作。カメラ映像・ステアリング値のLOG取得。

![course img](https://raw.githubusercontent.com/DaaasukeMurata/rc_w_dl/images/course.jpg)

# 2. 学習用データへの加工

## カメラ画像の加工パラメータ調整（ros/rc_image_w_tf/scripts/image_process/）

カメラ映像を、edge抽出・直線検出を行い単純化するためのパラメータ調整。

```
$ rosrun rc_image_w_tf rc_line_detect.py _image:="/usb_cam_node/image_raw" _gui:=True
```

調整用のwindowが立ち上がる。映像を見ながら調整。

![GUI Tool img](https://raw.githubusercontent.com/DaaasukeMurata/rc_w_dl/images/img_process.jpg)

| Category      | Parameter     | Description      |
|:--------------|:--------------|:-----------------|
| System        | pre_resize    | 入力画像の解像度削減。画像処理の低減に使用 |
|               | thining       | frameの間引き。処理が追いつかない場合に使用 |
|               | color_filter  | 色抽出フィルタ |
|               | to_gray       | モノクロに変換 |
|               | blur          | ぼかし |
|               | detect_edge   | 輪郭検出 |
|               | image_mask    | 直線検出に用いる画像マスク |
|               | detect_line   | 直線検出 |
|               | final_resize  | 出力画像の解像度削減。学習処理の低減に使用 |
|               | mono_output   | monoで画像出力　|
| color         | -             | 色抽出フィルタの上限下限 |
| blur          | -             | ガウシアンフィルタのフィルタサイズ |
| edge          | canny         | Canny Edge Detectとそのパラメータ |
|               | findContours  | 輪郭検出。edgeを太くすのに使用 |
| houghline     | -             | HoughLinesPのパラメータ |
| extrapolation_lines| -        | 白線と認識する角度の範囲 |

決めたパラメータは、ros/rc_image_w_tf/scripts/param_server.pyに値設定。

## 学習データの作成（create_testdata）

```
$ ./create_learn_data.sh [in_directory or xxx.bag] [outdir]
```

上で決めたパラメータで映像加工、ステアリング値と一緒にしてbinary array（npyファイル）に格納。

映像frame毎に、

- 加工後の画像
- ステアリング値
- サーボ値（今回使用しない）
- 検出した白線の位置

を格納。

**[その他 script]**

複数npyデータの結合
```
$ python concat_train_data.py [in_directory_path] [out.npy]
```

npyデータのsummery表示
```
$ python check_steer_num.py [in.npy]

-- output sample --
file :learndata/traind_normal_w_line_linemeta.npy
shape:(22911, 28802)
frame of detecting FRONT line: 18085 / 22911
frame of detecting LEFT  line: 2205 / 22911

steer:
  [  0-  9:    0]     [ 10- 19:    0]     [ 20- 29:    0]  
  [ 30- 39: 3773]     [ 40- 49:  121]     [ 50- 59:  147]  
  [ 60- 69:  419]     [ 70- 79:  664]     [ 80- 89:  859]  
  [ 90- 90:15327]  
  [ 91-100:  241]     [101-110:  199]     [111-120:  294]  
  [121-130:   86]     [131-140:  107]     [141-150:  674]  
  [151-160:    0]     [161-170:    0]     [171-180:    0]  

speed:
  [  0-  9:    0]     [ 10- 19:    0]     [ 20- 29:    0]  
  [ 30- 39:    0]     [ 40- 49:    0]     [ 50- 59:    0]  
  [ 60- 69:    0]     [ 70- 79:    0]     [ 80- 89:22906]  
  [ 90- 90:    0]  
  [ 91-100:    5]     [101-110:    0]     [111-120:    0]  
  [121-130:    0]     [131-140:    0]     [141-150:    0]  
  [151-160:    0]     [161-170:    0]     [171-180:    0]  
```

停止中のデータ削除
```
$ ython remove_straight_data.py [in.npy] [out.npy]
```

# 3. TensorFlowで学習（learning/）

```
$ python learning.py
```

## Training

- Input Data : 22911 sample、並びはランダムに変更

- Train Param : AdamOptimizer、学習率0.001、mini batch 64 data/batch、dropout 0.5

- Model : 下記3パターン試したが、ラジコンの挙動はあまり変わらず
    - 加工後の画像のみ
    - 加工後の画像 + 検出した白線の画像
    - 加工後の画像 + 検出した白線の画像 + 検出した白線の位置（座標）  

    3つめのModelは下図。"input"が画像データ、"input_line_meta"が白線位置（座標）

    加工後の画像と、検出した白線の画像はtf.nn.separable_conv2d[（解説記事 - Qiita）](http://qiita.com/YusukeSuzuki@github/items/0764d15b9d0b97ec1e16)を使い、画像ごとに重みが設定できるように。

    検出した白線位置 > 検出した白線の画像 > 加工後の画像 の順で判断してくれることを期待。
    
    ![tf model img](https://raw.githubusercontent.com/DaaasukeMurata/rc_w_dl/images/tensorboard_graphs.png)

## Result

学習に使ったデータとは別に、正誤判定用データでの結果がaccuracy。

開始早々に収束。一応全データ入力したが、サンプル数は過剰と思われる。

![tf result img](https://raw.githubusercontent.com/DaaasukeMurata/rc_w_dl/images/tensorboard_result_v3.png)


# 4. ラジコン制御（ros/rc_image_w_tf/）

```
$ roslaunch rc_image_w_tf rc_steer_w_tf.launch
```

- カメラの映像を、学習データ作成時に決めたParameterを用いて、リアルタイムに変換。学習済みのModelでステアリング値を取得

- 速度は一定

[走行結果]

コーナーは曲がるが、直線ですぐ壁にぶつかる。

[Movie(GoogleDrive)](https://drive.google.com/open?id=0B0mNEspU9cAiS2N3SVBUQlBHNjQ)
