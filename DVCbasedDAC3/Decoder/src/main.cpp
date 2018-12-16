
#include <iostream>

#include "decoder.h"

using namespace std;

int main(int argc, char** argv)
{
  if (argc == 1) {
    cerr << "[wz varBitstream file][key frame file][original video file]" << endl;
    return 1;
  }
  else if (argc == 4) {
    cout << endl;
    cout << "Distributed video coding using distributed arithmetic coding" << endl;
    cout << endl;

    Decoder* decoder = new Decoder(argv);

    decoder->decodeWZframe();

    cout << endl;
    cout << "Bye" << endl;
    cout << endl;
  }
  system("pause");
  return 0;
}

