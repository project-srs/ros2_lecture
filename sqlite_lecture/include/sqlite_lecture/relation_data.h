#include <string>
#include <vector>
#include <map>

class RelationData
{
public:
  RelationData()
  {
  }

  void increment(void)
  {
    line_size_++;
    for (auto &c : data_)
    {
      c.second.push_back("");
    }
  }

  void setAtBack(std::string column_name, std::string value)
  {
    // new
    if (data_.count(column_name) == 0)
    {
      std::vector<std::string> column;
      column.resize(line_size_);
      data_[column_name] = column;
    }

    if (data_[column_name].empty())
    {
      printf("empty %s\n", column_name.c_str());
      return;
    }
    data_[column_name].back() = value;
  }

  std::string get(std::string column_name, size_t line) const
  {
    if (!checkSize())
    {
      return "";
    }
    if (data_.count(column_name) == 0)
    {
      printf("not found %s\n", column_name.c_str());
      return "";
    }
    if (line >= data_.at(column_name).size())
    {
      printf("no data %s %lu (should < %lu)\n", column_name.c_str(), line, data_.at(column_name).size());
      return "";
    }
    return data_.at(column_name)[line];
  }

  size_t size(void) const
  {
    if (!checkSize())
    {
      return 0;
    }
    return line_size_;
  }

  void print(void) const
  {
    if (!checkSize())
    {
      return;
    }
    printf("======table======\n");
    for (auto c : data_)
    {
      printf("[%s]: ", c.first.c_str());
      for (auto v : c.second)
      {
        printf("%s, ", v.c_str());
      }
      printf("\n");
    }
    printf("=================\n");
  }

private:
  bool checkSize(void) const
  {
    bool size_is_ok = true;
    for (auto c : data_)
    {
      if (line_size_ != c.second.size())
      {
        size_is_ok = false;
        printf("%s is %lu (expected %lu)\n", c.first.c_str(), c.second.size(), line_size_);
      }
    }
    return size_is_ok;
  }

  size_t line_size_{0};
  std::map<std::string, std::vector<std::string>> data_;
};
