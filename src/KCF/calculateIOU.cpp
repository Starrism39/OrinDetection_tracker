#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include "KCF/calculateIOU.h"

// ��������
double calculateIoU(const std::vector<int>& box1, const std::vector<int>& box2);
void readBoxData(const std::string& fileName, std::vector<std::vector<int>>& boxes);
void writeIoUResults(const std::vector<double>& iouResults, const std::string& outputFileName);

void calculateError() {
    // ��ȡ������?
    std::vector<std::vector<int>> boxes1, boxes2;
    readBoxData("output.txt", boxes1);
    readBoxData("region.txt", boxes2);

    // ����IOU���洢���?
    std::vector<double> iouResults;
    for (size_t i = 0; i < boxes1.size() && i < boxes2.size(); ++i) {
        double iou = calculateIoU(boxes1[i], boxes2[i]);
        iouResults.push_back(iou);
    }

    // �����д���µ�txt�ļ�
    writeIoUResults(iouResults, "iou_results.txt");

}

// ����IOU
double calculateIoU(const std::vector<int>& box1, const std::vector<int>& box2) {
    int x1 = std::max(box1[0], box2[0]);
    int y1 = std::max(box1[1], box2[1]);
    int x2 = std::min(box1[0] + box1[2], box2[0] + box2[2]);
    int y2 = std::min(box1[1] + box1[3], box2[1] + box2[3]);

    int intersectionArea = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int unionArea = box1[2] * box1[3] + box2[2] * box2[3] - intersectionArea;

    return static_cast<double>(intersectionArea) / static_cast<double>(unionArea);
}

// ��txt�ļ��ж�ȡ������?
void readBoxData(const std::string& fileName, std::vector<std::vector<int>>& boxes) {
    std::ifstream inputFile(fileName);
    std::string line;

    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        std::vector<int> box;
        int value;
        char delimiter;

        while (iss >> value) {
            box.push_back(value);
            iss >> delimiter; // ��ȡ����
        }

        boxes.push_back(box);
    }

    inputFile.close();
}

// ��IOU���д���µ�txt�ļ�
void writeIoUResults(const std::vector<double>& iouResults, const std::string& outputFileName) {
    std::ofstream outputFile(outputFileName);

    for (const auto& iou : iouResults) {
        outputFile << iou << std::endl;
    }

    outputFile.close();
}
