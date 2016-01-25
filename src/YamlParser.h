/*


*/

#ifndef __YAML_PARSER_H__
#define __YAML_PARSER_H__

#include <iostream>
#include <strstream>
#include <string>
#include <vector>
#include <map>
#include <stdio.h>
//#include <yaml.h>
#include "yaml.h"


enum YamlNodeType{
	YAML_NONE, YAML_SCALAR, YAML_MAPPING, YAML_SEQUENCE,
};

class YamlScalar;
class YamlMapping;
class YamlSequence;
class YamlNode;
class YamlDocument;

class YamlNode
{
public:
	YamlNode();
	~YamlNode();

	void addChild(YamlNode *n);
	void print();

	YamlScalar *toScalar();
	YamlMapping *toMapping();
	YamlSequence *toSequence();

	bool isScalar();
	bool isMapping();
	bool isSequence();
    bool isRoot();

	bool equalTo(char *str);

public:
	enum YamlNodeType type;
	std::vector<YamlNode *> childNodes;
	YamlNode *parentNode;

};


class YamlScalar:  YamlNode
{
public:
	YamlScalar(char *str);
	~YamlScalar();

	void setValue(char *str);
	void print();

	std::string toString();
	int toInteger();
	double toDouble();

public:
	std::string value;

};


class YamlMapping:  YamlNode
{
public:
	YamlMapping();
	~YamlMapping();

	void insertValue(char *k, YamlNode *v);
	void print();

	YamlNode *getValue(char *k);

	bool equalTo(char *k, char *val);
	YamlScalar *getScalar(char *k);
	YamlMapping *getMapping(char *k);
	YamlSequence *getSequence(char *k);

	YamlNode * operator[](char *k) { return getValue(k); }

public:
	std::map<std::string, YamlNode *> values;

};

class YamlSequence:  YamlNode
{
public:
	YamlSequence();
	~YamlSequence();

	void appendValue(YamlNode *v);
	void print();

	int size();
	YamlNode *at(int x);
	YamlScalar *getScalarAt(int x);
	YamlMapping *getMappingAt(int x);
	YamlSequence *getSequenceAt(int x);

	YamlNode * operator[](int n) { return value[n]; }

public:
	std::vector<YamlNode *> value;

};



class YamlDocument
{
public:
	YamlDocument();
	~YamlDocument();

public:
	bool load(char *fname);
	void print();
	bool empty();

private:
	YamlSequence *ParseYamlSequence();
	YamlMapping *ParseYamlMapping();
	YamlNode *ParseYamlDoc();

public:
	YamlNode *root;

private:
	yaml_parser_t parser;
	yaml_event_t event;
};



#endif