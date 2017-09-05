#pragma once

#include <vector>
#include <bits/stringfwd.h>
#include <algorithm>

using namespace std;

// A simple command line parser. Taken from:
// https://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
class InputParser {
public:
  InputParser(int &argc, char **argv)
  {
    for (auto i = 1; i < argc; ++i)
    {
      this->tokens.push_back(string(argv[i]));
    }
  }
  const string& getCmdOption(const string &option) const
  {
    auto itr = find(this->tokens.begin(), this->tokens.end(), option);

    if (itr != this->tokens.end() && ++itr != this->tokens.end())
    {
      return *itr;
    }

    static const string empty_string("");

    return empty_string;
  }

  bool cmdOptionExists(const string &option) const
  {
    return find(this->tokens.begin(), this->tokens.end(), option) != this->tokens.end();
  }

private:
  vector<string> tokens;
};
