/**
 * A collection of handy C++ functions.
 *
 * @author Jacob Perron <jperron@sfu.ca>
 */

#ifndef JACOBUTILS_H
#define JACOBUTILS_H

#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <fstream>
#include <sys/time.h>
#include <dirent.h>

/** Return a list of all files in a given directory with the provided suffix.
 *  E.g. get_files("home", ".txt") returns a list of all '.txt' files in the directory 'home'
 *
 *  @param directory : The directory path to search
 *  @param suffix : The end of the file names
 *  @return A list of file names in \p directory ending with \p suffix
 */
static std::vector<std::string> get_files(const std::string& directory, const std::string& suffix){ 
    std::vector<std::string> result;
    DIR* dir = opendir(directory.c_str());

    if (!dir) return result; //empty result

    dirent* entry;
    while (entry = readdir(dir)) {
        std::string name = entry->d_name;
        if (name.size() >= suffix.size() &&
            equal(suffix.rbegin(), suffix.rend(), name.rbegin())) {
            result.push_back(name);
        }
    }

    return result;
}

/** Split a string into tokens given a delimiter.
 *  
 *  @param s : The string to split
 *  @param delim: The delimiter to use
 *  @return A list of substrings
 */
static std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    //split(s, delim, elems);
    return elems;
}

/** Trim leading and trailing whitespace from a string.
 *  @param str : The string to trim.
 *  @param whitespace : The type of whitespace to remove.
 *  @return The trimmed string.
 */
static std::string trim(const std::string& str, const std::string& whitespace = " \t") {
    const size_t strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos)
        return ""; // no content
    
    const size_t strEnd = str.find_last_not_of(whitespace);
    const size_t strRange = strEnd - strBegin + 1;
    
    return str.substr(strBegin, strRange);
}

/** Parse a simple configuration file.
 *  Key-value pairs are separated by \p delim character. One per line.
 *  Comments are preceded by \p comment character.
 *  Comments and key-value pairs cannot be on the same line.
 *  Leading and trailing white space is ignored.
 *
 *  @param path : The path to the configuration file.
 *  @param delim : The key-value delimiter
 *  @param comment : The character preceding comments
 *  @return A list of key-value pairs where keys and values are strings.
 */
static std::map<std::string, std::string> parse_config(const std::string& path,
                                             const char delim = ':',
                                             const char comment = '#') {
    std::map<std::string, std::string> config;
    std::ifstream config_file(path.c_str());
    std::string line;
    while (std::getline(config_file, line)) {
        line = trim(line);
        
        // Check if empty line or comment
        if (line.length() < 1 || line[0] == comment)
            continue;

        std::string key = trim(line.substr(0, line.find(delim)));
        std::string val = trim(line.substr(line.find(delim)+1, line.size()));
        config[key] = val;
    }

    return config;
}

/** Conversion from double to std::string.
 */
static std::string dtos(double d) {
    std::ostringstream strs;
    strs << d;
    return strs.str(); 
} 

/** Conversion from int to std::string.
 */
static std::string itos(int d) {
    std::ostringstream strs;
    strs << d;
    return strs.str(); 
} 

/** Get a timestamp for the current time in micro-seconds.
 */
typedef unsigned long long timestamp_t;
static timestamp_t get_timestamp() {
    struct timeval now;
    gettimeofday(&now, NULL);
    return now.tv_usec + (timestamp_t) now.tv_sec * 1000000;
}
#endif
