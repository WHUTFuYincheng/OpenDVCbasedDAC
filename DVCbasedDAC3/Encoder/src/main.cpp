
#include <iostream>

#include "encoder.h"

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 10) {
    cerr << endl;
    cerr << "Usage: Encoder ";
    cerr << "[WZ QP] [key QP] [width] [height] ";
    cerr << "[frame number] [GOP level] ";
    cerr << "[input file] [output file] [recon file]" << endl;
    cerr << endl;
  }
  else {
    cout << endl;
    cout << "Distributed video coding using distributed arithmetic coding" << endl;
    cout << endl;

    Encoder* encoder = new Encoder(argv);

    encoder->encodeKeyFrame();

    encoder->encodeWzFrame();

    cout << endl;
    cout << "Bye" << endl;
    cout << endl;
  }
  system("pause");
  return 0;
}

