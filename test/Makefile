# コンパイラ設定
CXX = g++
CXXFLAGS = -Wall -Wextra -I../include -pthread

# リンクするライブラリ (fmtが必要な場合)
LDFLAGS = -lfmt

# 生成する実行ファイル名
TARGET = tcp_thread_test

# ソースファイルのリスト (ここに追加が必要です！)
SRCS = tcp_thread_test.cpp \
       ../src/network/tcp/tcp_thread.cpp \
       ../src/network/tcp/tcp_socket.cpp \
       ../src/network/resolve_hostname.cpp \
       ../src/json/json_client.cpp \
       ../src/json/json_codec.cpp

# オブジェクトファイルのリスト生成 (SRCSから自動生成)
OBJS = $(SRCS:.cpp=.o)

# ビルドルール
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# .cpp -> .o のコンパイルルール
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)