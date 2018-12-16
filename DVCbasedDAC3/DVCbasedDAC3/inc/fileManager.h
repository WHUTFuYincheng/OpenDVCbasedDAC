
#ifndef COMMON_INC_FILEMANAGER_H
#define COMMON_INC_FILEMANAGER_H

#include <string>
#include <map>

#include <cstdio>

using std::string;
using std::map;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
class File
{
public:
  File(const string& fileName)
  {
    _fileName   = fileName;
    _fileHandle = 0;
  };

  bool openFile(const string& type)
  {
    _fileHandle = fopen(_fileName.c_str(), type.c_str());

    return _fileHandle ? true : false;
  };

  void closeFile() { fclose(_fileHandle); };

  const string& getFileName()   { return _fileName; };
  FILE*         getFileHandle() { return _fileHandle; };

private:
  string  _fileName;
  FILE*   _fileHandle;
};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
class FileManager
{
public:
  static FileManager* getManager()
  {
    static FileManager* _manager = new FileManager();

    return _manager;
  };

  File* addFile(const string& name, const string& fileName)
  {
    File* file = new File(fileName);

    _fileMap[name] = file;

    return file;
  };

  File* getFile(const string& name)
  {
    if (_fileMap.find(name) != _fileMap.end())
      return _fileMap[name];
    else
      return 0;
  };

private:
  FileManager() {};

  map<const string, File*> _fileMap;
};

#endif // COMMON_INC_FILEMANAGER_H

