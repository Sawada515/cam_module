# コンパイラとフラグ設定
CXX      = g++
CXXFLAGS = -Wall -Wextra -I../include -pthread

# ★追加: リンクするライブラリ
# spdlogが依存するfmtライブラリをリンクします
LIBS     = -lfmt

# 出力する実行ファイル名
TARGET   = tcp_thread_test

# コンパイル対象のソースファイル一覧
SRCS     = tcp_thread_test.cpp \
           ../src/network/tcp_thread.cpp \
           ../src/network/tcp_socket.cpp \
           ../src/network/resolve_hostname.cpp

# オブジェクトファイルのリスト生成
OBJS     = $(SRCS:.cpp=.o)

# 1. リンク: 全ての .o を繋げて実行ファイルを作る
# ★変更: 末尾に $(LIBS) を追加
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# 2. コンパイル: 各 .cpp から .o を作るルール
.cpp.o:
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 生成ファイルの削除用
clean:
	rm -f $(OBJS) $(TARGET)