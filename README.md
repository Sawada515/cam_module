# 標準課題2用プログラム
Webカメラで基板の動画を撮影・UDP送信用のプログラム
抵抗値を画像処理と機械学習を用いて推測

## 想定動作環境
Raspberry Pi 5<br>
Jetson <br>
Webカメラ (V4L2対応)

## 開発環境
Linux kali 6.17.10+kali-amd64 #1 SMP PREEMPT_DYNAMIC Kali 6.17.10-1kali1 (2025-12-08) x86_64 GNU/Linux

## システム構成図
<img width="1544" height="935" alt="Image" src="https://github.com/user-attachments/assets/358a0abe-a72c-4d55-b700-44003ff9fa7c" />

## コンパイル
1. ビルド用ディレクトリを作成<br>
2. CMakeでMakefileを生成<br>
3. Makefileを実行

```terminal
$ mkdir build && cd $_
$ cmake ..
$ make
```
cmakeの際の指定オプション<br><br>
```terminal
cmake -DCMAKE_BUILD_TYPE=type ..
```
| type | コンパイルオプション |
| --- | --- |
| Release | -O3 -DNDEBUG |
| Debug | -g |

## 動作デモ
デスクトップアプリケーションは本リポジトリに含まれておりません
<img width="1700" height="1047" alt="Image" src="https://github.com/user-attachments/assets/ca9a8ba1-1a9a-434f-8b81-1e8c20b2be67" />
