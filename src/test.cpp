
#include <iostream>
#include <string>    // useful for reading and writing

#include <fstream>   // ifstream, ofstream

int main()
{
  std::string filename = "evtime_log.txt";

  std::ofstream writing_file;
  writing_file.open(filename, std::ios::out);

  //std::cout << "writing " << filename << "..." << std::endl;

  _Float32 i=1.873;
  for (int j = 0; j < 10; j++)
  {
    /* code */
    writing_file << i << std::endl;
    i*=3;
  }
  
  return 0;
  
  

}

