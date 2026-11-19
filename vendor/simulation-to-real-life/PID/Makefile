CXX ?= g++
CXXFLAGS = -std=c++17 -O3 -Wall -Wextra -pedantic
OPENMP_FLAG = -fopenmp

XLSX_INC ?= -I C:/libxlsxwriter-install/include
XLSX_LIB ?= -L C:/libxlsxwriter-install/lib -lxlsxwriter

ifeq ($(OS),Windows_NT)
    BIN_EXT = .exe
    LDFLAGS = -lws2_32 -lpthread $(XLSX_LIB)
    RAYLIB_FLAGS = -I C:/raylib/raylib/src -L C:/raylib/raylib/src -lraylib -lopengl32 -lgdi32 -lwinmm $(XLSX_LIB)
    RM = del /Q /F
else
    BIN_EXT =
    LDFLAGS = -pthread $(XLSX_LIB)
    RAYLIB_FLAGS = -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 $(XLSX_LIB)
    RM = rm -f
endif

TRAIN_BIN = train$(BIN_EXT)
BENCH_BIN = benchmark$(BIN_EXT)
EXPORT_BIN = export_gains$(BIN_EXT)
VISUAL_BIN = visualizer$(BIN_EXT)
RECORD_BIN = record$(BIN_EXT)

HEADERS = Types.hpp SimConfig.hpp Raycast.hpp Track.hpp StraightGuide.hpp PidController.hpp Robot.hpp GraphExportHelper.hpp MotorModel.hpp ../firmware/robot_fw/wheel_pid.h

# The run targets are prefixed run- because on Linux/WSL BIN_EXT is empty, so the binary
# itself is named train / benchmark. A phony target of the same name would override the
# build rule and make would try to run a binary it never built.
.PHONY: all visual run-train run-benchmark export clean replay

all: $(TRAIN_BIN) $(BENCH_BIN) $(EXPORT_BIN) $(VISUAL_BIN)

$(TRAIN_BIN): train.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(OPENMP_FLAG) $(XLSX_INC) train.cpp -o $(TRAIN_BIN) $(LDFLAGS)

$(BENCH_BIN): benchmark.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(XLSX_INC) benchmark.cpp -o $(BENCH_BIN) $(LDFLAGS)

$(EXPORT_BIN): export_gains.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) export_gains.cpp -o $(EXPORT_BIN)

# Headless run saved as an interactive HTML replay in videos/ (no xlsxwriter or raylib needed).
$(RECORD_BIN): record.cpp replay_player.inc $(HEADERS)
	$(CXX) $(CXXFLAGS) record.cpp -o $(RECORD_BIN)

replay: $(RECORD_BIN)
	./$(RECORD_BIN) --open

$(VISUAL_BIN): visualizer.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(XLSX_INC) visualizer.cpp -o $(VISUAL_BIN) $(RAYLIB_FLAGS)

run-train: $(TRAIN_BIN)
	./$(TRAIN_BIN)

run-benchmark: $(BENCH_BIN)
	./$(BENCH_BIN)

export: $(EXPORT_BIN)
	./$(EXPORT_BIN) best_pid_model.txt pid_gains.h

visual: $(VISUAL_BIN)
	./$(VISUAL_BIN)

clean:
	$(RM) $(RECORD_BIN) $(TRAIN_BIN) $(BENCH_BIN) $(EXPORT_BIN) $(VISUAL_BIN) pid_gains.h benchmark_results.csv training_log.csv *.o *.exe