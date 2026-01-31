# コンパイラ設定
CXX      := g++

CXXFLAGS := -std=c++17 -O3 -Wall -Wextra

# インクルードパス
# テスト実行場所(test/camera)から見て、ヘッダは ../../include にある
INCLUDES := -I../../include
CXXFLAGS += $(INCLUDES)

# ライブラリ設定
# OpenCV
CXXFLAGS += $(shell pkg-config --cflags opencv4)
LDLIBS   += $(shell pkg-config --libs opencv4)

# システムライブラリ (スレッド、ログ関連)
# 環境に合わせて -lfmt -lspdlog は必要に応じて調整してください
LDLIBS   += -lpthread -lfmt -lspdlog

# 生成する実行ファイル名
TARGET   := get_frame

# ソースファイル
# 1. テストプログラム本体 (カレントディレクトリ)
# 2. カメラライブラリの実装 (../../src/camera)
SRCS     := get_frame.cpp \
            ../../src/camera/capture_thread.cpp \
            ../../src/camera/v4l2_capture.cpp

# オブジェクトファイルの定義
OBJS     := $(SRCS:.cpp=.o)

# ターゲット定義
.PHONY: all clean

all: $(TARGET)

# リンクルール
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

# コンパイルルール (カレントディレクトリの .cpp 用)
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# コンパイルルール (../../src/camera 内の .cpp 用)
../../src/camera/%.o: ../../src/camera/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# クリーンアップ
# 生成されたオブジェクトファイルと実行ファイルを削除
clean:
	rm -f $(TARGET) *.o ../../src/camera/*.o
