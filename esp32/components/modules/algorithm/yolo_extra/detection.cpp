#include <iostream>
#include <vector>

class Detection
{
public:
      Detection(const std::vector<float> &tlwh, float confidence, const std::string &class_name, const std::vector<float> &feature) : tlwh_(tlwh), confidence_(confidence), class_name_(class_name), feature_(feature) {}

      std::string get_class() const
      {
            return class_name_;
      }

      std::vector<float> to_tlbr()
      {
            std::vector<float> ret(4);
            ret[0] = tlwh_[0];
            ret[1] = tlwh_[1];
            ret[2] = tlwh_[0] + tlwh_[2];
            ret[3] = tlwh_[1] + tlwh_[3];
            return ret;
      }

      std::vector<float> to_xyah()
      {
            std::vector<float> ret(4);
            ret[0] = tlwh_[0] + tlwh_[2] / 2;
            ret[1] = tlwh_[1] + tlwh_[3] / 2;
            ret[2] = tlwh_[2] / tlwh_[3];
            ret[3] = tlwh_[3];
            return ret;
      }

private:
      std::vector<float> tlwh_;
      float confidence_;
      std::string class_name_;
      std::vector<float> feature_;
};
