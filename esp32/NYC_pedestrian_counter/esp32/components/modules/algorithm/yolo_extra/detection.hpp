#ifndef DETECTION_HPP
#define DETECTION_HPP

#include <vector>
#include <string>

class Detection
{
public:
      Detection(const std::vector<float> &tlwh, float confidence, const std::string &class_name, const std::vector<float> &feature);
      std::string get_class() const;
      std::vector<float> to_tlbr() const;
      std::vector<float> to_xyah() const;

private:
      std::vector<float> tlwh_;
      float confidence_;
      std::string class_name_;
      std::vector<float> feature_;
};

#endif // DETECTION_HPP