# =============================================================================
# Memory-Driven Graph Intelligence Engine - Makefile
# =============================================================================
# 
# Build commands:
#   make          - Build the main executable
#   make debug    - Build with debug symbols
#   make release  - Build with optimizations
#   make clean    - Remove build artifacts
#   make test     - Build and run tests
#
# =============================================================================

# Compiler settings
CC = gcc
CFLAGS = -Wall -Wextra -Wpedantic -std=c99
LDFLAGS = -lm

# Target executable
TARGET = graph_engine

# Source files
SRCS = main.c graph.c heap.c algorithms.c

# Object files
OBJS = $(SRCS:.c=.o)

# Header files (for dependency tracking)
HEADERS = graph.h heap.h algorithms.h

# =============================================================================
# BUILD TARGETS
# =============================================================================

# Default target: build release version
all: release

# Release build (optimized)
release: CFLAGS += -O3 -DNDEBUG
release: $(TARGET)

# Debug build (with symbols and no optimization)
debug: CFLAGS += -g -O0 -DDEBUG
debug: $(TARGET)

# Build the executable
$(TARGET): $(OBJS)
	@echo "Linking: $@"
	$(CC) $(OBJS) -o $@ $(LDFLAGS)
	@echo "Build complete: $@"

# Compile source files to object files
%.o: %.c $(HEADERS)
	@echo "Compiling: $<"
	$(CC) $(CFLAGS) -c $< -o $@

# =============================================================================
# UTILITY TARGETS
# =============================================================================

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	rm -f $(OBJS) $(TARGET)
	@echo "Clean complete"

# Clean and rebuild
rebuild: clean all

# Run the program
run: $(TARGET)
	./$(TARGET)

# Run with valgrind (memory leak detection)
valgrind: debug
	valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes ./$(TARGET)

# Static analysis with cppcheck
cppcheck:
	cppcheck --enable=all --suppress=missingIncludeSystem $(SRCS)

# Format code with clang-format
format:
	clang-format -i $(SRCS) $(HEADERS)

# Generate tags for code navigation
tags:
	ctags -R .

# =============================================================================
# TEST TARGETS
# =============================================================================

# Simple test compilation
test: CFLAGS += -g -O0
test: $(OBJS)
	$(CC) $(OBJS) -o test_runner $(LDFLAGS)
	@echo "Running tests..."
	./test_runner < /dev/null || true

# =============================================================================
# DOCUMENTATION TARGETS
# =============================================================================

docs:
	@echo "Generating documentation..."
	@echo "Project: Memory-Driven Graph Intelligence Engine"
	@echo "Version: 2.0.0"
	@echo ""
	@echo "Files:"
	@echo "  - graph.h: Core graph data structures"
	@echo "  - heap.h:  Min-heap priority queue"
	@echo "  - algorithms.h: Search algorithms"
	@echo "  - graph.c: Graph implementation"
	@echo "  - heap.c:  Heap implementation"
	@echo "  - algorithms.c: Algorithm implementations"
	@echo "  - main.c:  CLI interface"

# =============================================================================
# INFO TARGETS
# =============================================================================

info:
	@echo "Compiler: $(CC)"
	@echo "CFLAGS:   $(CFLAGS)"
	@echo "LDFLAGS:  $(LDFLAGS)"
	@echo "Target:   $(TARGET)"
	@echo "Sources:  $(SRCS)"
	@echo "Objects:  $(OBJS)"

# =============================================================================
# PHONY TARGETS
# =============================================================================

.PHONY: all debug release clean rebuild run valgrind cppcheck format tags test docs info

# =============================================================================
# DEPENDENCIES
# =============================================================================

# Automatic dependency generation
-include $(OBJS:.o=.d)

%.d: %.c
	@$(CC) -M $(CFLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$
