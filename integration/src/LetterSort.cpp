// longest_word_user_input.cpp
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <algorithm>

std::unordered_map<char, int> countLetters(const std::string& str) {
    std::unordered_map<char, int> freq;
    for (char c : str) {
        freq[c]++;
    }
    return freq;
}

bool canFormWord(const std::string& word, const std::unordered_map<char, int>& letterFreq) {
    auto wordFreq = countLetters(word);
    for (const auto& [c, count] : wordFreq) {
        if (letterFreq.find(c) == letterFreq.end() || letterFreq.at(c) < count) {
            return false;
        }
    }
    return true;
}

int main() {
    std::string letters;
    std::cout << "Enter 6 letters: ";
    std::cin >> letters;

    if (letters.length() != 7) {
        std::cerr << "Error: Please enter exactly 6 letters.\n";
        return 1;
    }

    std::transform(letters.begin(), letters.end(), letters.begin(), ::tolower);
    auto letterFreq = countLetters(letters);

    std::ifstream dictFile("/usr/share/dict/words");
    if (!dictFile) {
        std::cerr << "Failed to open dictionary file.\n";
        return 1;
    }

    std::string word;
    std::string longestWord = "";

    while (std::getline(dictFile, word)) {
        std::transform(word.begin(), word.end(), word.begin(), ::tolower);
        if (word.length() > letters.length()) continue;
        if (canFormWord(word, letterFreq) && word.length() > longestWord.length()) {
            longestWord = word;
        }
    }

    std::cout << "Longest word that can be formed: " << longestWord << "\n";
    return 0;
}
