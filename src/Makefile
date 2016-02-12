robo : CUtils.o GROBO.o robo.o SerialPort.o SerialRobot.o stdafx.o YamlParser.o
	g++ -Wall -DNDEBUG -D_USRDLL -DGR001_EXPORTS -DYAML_DECLARE_STATIC -lpthread -lyaml -o robo *.o
CUtils.o : CUtils.cpp
	g++ -c -Wall -DNDEBUG -D_USRDLL -DGR001_EXPORTS -DYAML_DECLARE_STATIC CUtils.cpp 
GROBO.o : GROBO.cpp
	g++ -c -Wall -DNDEBUG -D_USRDLL -DGR001_EXPORTS -DYAML_DECLARE_STATIC GROBO.cpp
robo.o : robo.cpp
	g++ -c -Wall -DNDEBUG -D_USRDLL -DGR001_EXPORTS -DYAML_DECLARE_STATIC robo.cpp 
SerialPort.o : SerialPort.cpp
	g++ -c -Wall -DNDEBUG -D_USRDLL -DGR001_EXPORTS -DYAML_DECLARE_STATIC SerialPort.cpp
SerialRobot.o : SerialRobot.cpp
	g++ -c -Wall -DNDEBUG -D_USRDLL -DGR001_EXPORTS -DYAML_DECLARE_STATIC SerialRobot.cpp
stdafx.o : stdafx.cpp
	g++ -c -Wall -DNDEBUG -D_USRDLL -DGR001_EXPORTS -DYAML_DECLARE_STATIC stdafx.cpp 
YamlParser.o : YamlParser.cpp
	g++ -c -Wall -DNDEBUG -D_USRDLL -DGR001_EXPORTS -DYAML_DECLARE_STATIC YamlParser.cpp
clean :
	rm *.o robo
