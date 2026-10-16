#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace cepton_ros
{

// Removes a field and compacts each point's binary representation.
// Returns false when the field is not present.
template <typename CloudMessage>
bool remove_point_field(CloudMessage& msg, const std::string& name)
{
  using Field = typename decltype(msg.fields)::value_type;
  auto field_it =
      std::find_if(msg.fields.begin(), msg.fields.end(), [&name](const Field& field) { return field.name == name; });
  if (field_it == msg.fields.end())
    return false;

  const size_t field_size = [&field_it]() {
    switch (field_it->datatype)
    {
      case Field::INT8:
      case Field::UINT8:
        return size_t{ 1 };
      case Field::INT16:
      case Field::UINT16:
        return size_t{ 2 };
      case Field::INT32:
      case Field::UINT32:
      case Field::FLOAT32:
        return size_t{ 4 };
      case Field::FLOAT64:
        return size_t{ 8 };
      default:
        throw std::invalid_argument("Unsupported PointCloud2 field datatype");
    }
  }() * field_it->count;
  const auto field_offset = field_it->offset;
  const auto new_point_step = msg.point_step - field_size;
  const auto new_row_step = msg.width * new_point_step;
  std::vector<uint8_t> data(new_row_step * msg.height);
  for (size_t row = 0; row < msg.height; ++row)
  {
    for (size_t column = 0; column < msg.width; ++column)
    {
      auto* destination = data.data() + row * new_row_step + column * new_point_step;
      const auto* source = msg.data.data() + row * msg.row_step + column * msg.point_step;
      std::memcpy(destination, source, field_offset);
      std::memcpy(destination + field_offset, source + field_offset + field_size,
                  msg.point_step - field_offset - field_size);
    }
  }
  msg.fields.erase(field_it);
  for (auto& field : msg.fields)
  {
    if (field.offset > field_offset)
      field.offset -= field_size;
  }
  msg.point_step = new_point_step;
  msg.row_step = new_row_step;
  msg.data.swap(data);
  return true;
}

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
