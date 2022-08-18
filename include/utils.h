//
// Created by hou on 2022/8/18.
//

#ifndef ORB_SLAM3_UTILS_H
#define ORB_SLAM3_UTILS_H

#include <string>
#include <iostream>
namespace ORB_SLAM3 {
    class Verbose {
    public:
        enum eLevel {
            VERBOSITY_QUIET = 0,
            VERBOSITY_NORMAL = 1,
            VERBOSITY_VERBOSE = 2,
            VERBOSITY_VERY_VERBOSE = 3,
            VERBOSITY_DEBUG = 4
        };

        static eLevel th;

    public:
        static void PrintMess(const std::string &str, eLevel lev) {
            if (lev <= th)
                std::cout << str << std::endl;
        }

        static void SetTh(eLevel _th) {
            th = _th;
        }
    };
}
#endif //ORB_SLAM3_UTILS_H
