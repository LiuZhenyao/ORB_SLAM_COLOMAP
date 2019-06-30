#pragma once

#include<iostream>

namespace vins_BinFile_reader {
    // This namespace define the functions used to read pose graph
    // generated from VINS-Mono, MYNT EYE supported
    void testFunc();
}

class BinFile_reader {
    public:
        BinFile_reader();                                                  // save_file_path{save_file_path_} {}
        ~BinFile_reader();

        // custom member function
        void testFunc();                                                                                    
        // bool reader();
        // bool saver();

    private:
        std::string input_file_path = "";
        std::string save_file_path = "";
};
