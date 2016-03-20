CXX      = g++
TARGET   = robo
INCLUDES = -I.
CXXFLAGS = -Wall $(INCLUDES)
MRAALIBS = -lmraa
LIBS = -lyaml -lpthread $(MRAALIBS)
SRCS = CUtils.cpp GROBO.cpp robo.cpp SerialPort.cpp SerialRobot.cpp stdafx.cpp YamlParser.cpp
OBJS := $(SRCS:.cpp=.o)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LIBS)

clean:
	rm -f $(TARGET) $(OBJS)

