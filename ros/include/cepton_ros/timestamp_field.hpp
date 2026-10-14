#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace cepton_ros
{

// Works with both sensor_msgs::PointCloud2 and pcl::PCLPointCloud2.
template <typename CloudMessage, typename Value>
void append_timestamp_field(CloudMessage& msg, const std::string& name, uint8_t datatype,
                            const std::vector<Value>& timestamps)
{
  if (timestamps.size() != static_cast<size_t>(msg.width) * msg.height)
  {
    throw std::invalid_argument("Point count and timestamp count differ");
  }
  typename decltype(msg.fields)::value_type field;
  field.name = name;
  field.offset = msg.point_step;
  field.datatype = datatype;
  field.count = 1;

  const uint16_t endian_probe = 1;
  const bool native_bigendian = *reinterpret_cast<const uint8_t*>(&endian_probe) == 0;
  const auto new_point_step = msg.point_step + sizeof(Value);
  const auto new_row_step = msg.width * new_point_step;
  std::vector<uint8_t> data(new_row_step * msg.height);
  for (size_t row = 0; row < msg.height; ++row)
  {
    for (size_t column = 0; column < msg.width; ++column)
    {
      auto* destination = data.data() + row * new_row_step + column * new_point_step;
      const auto* source = msg.data.data() + row * msg.row_step + column * msg.point_step;
      std::memcpy(destination, source, msg.point_step);
      std::memcpy(destination + field.offset, &timestamps[row * msg.width + column], sizeof(Value));
      if (native_bigendian != static_cast<bool>(msg.is_bigendian))
      {
        std::reverse(destination + field.offset, destination + field.offset + sizeof(Value));
      }
    }
  }
  msg.fields.push_back(field);
  msg.point_step = new_point_step;
  msg.row_step = new_row_step;
  msg.data.swap(data);
}

}  // namespace cepton_ros
