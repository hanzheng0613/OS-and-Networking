CXX = g++
CXXFLAGS = -std=c++17 -g
LDFLAGS = -lboost_system
TARGET = shm
SRC = shm.cpp

all: $(TARGET)

$(TARGET): $(SRC)
    $(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

run: $(TARGET)
    ./$(TARGET)

clean:
    rm -f $(TARGET)
