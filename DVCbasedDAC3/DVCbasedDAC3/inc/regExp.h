
#ifndef COMMON_INC_REGEXP_H
#define COMMON_INC_REGEXP_H

#include <regex.h>
#include <string>

using std::string;

class RegExp
{
public:
  static RegExp* getInst()
  {
    static RegExp* _inst = new RegExp();

    return _inst;
  };

  bool match(char* src, const char* pattern, string& result)
  {
    if (regcomp(&_regex, pattern, REG_EXTENDED))
      return false;

    if (regexec(&_regex, src, 2, _matched, 0) == 0) {
      int so = _matched[1].rm_so;
      int eo = _matched[1].rm_eo;
      string str(src);

      result = str.substr(so, eo-so);
    }
    else
      return false;

    regfree(&_regex);

    return true;
  };

private:
  RegExp() {};

  regex_t     _regex;
  regmatch_t  _matched[2];
};

#endif // COMMON_INC_REGEXP_H

