#include "map/identity_store.hpp"

#include "platform/secure_storage.hpp"
#include "platform/settings.hpp"

#include "base/string_utils.hpp"

#include "coding/base64.hpp"

#include <algorithm>
#include <array>
#include <random>
#include <string>
#include <string_view>

namespace
{
constexpr char kDeviceIdKey[] = "Explore.DeviceId";
constexpr char kUsernameKey[] = "Explore.Username";

// Convert RFC4648 base64 to URL-safe base64 (no padding, - and _ instead of + and /).
std::string ToUrlSafeBase64(std::string s)
{
  for (char & ch : s)
  {
    if (ch == '+')
      ch = '-';
    else if (ch == '/')
      ch = '_';
  }
  // strip '=' padding
  while (!s.empty() && s.back() == '=')
    s.pop_back();
  return s;
}
}  // namespace

std::string IdentityStore::GetOrCreateDeviceId()
{
  std::string deviceId;
  platform::SecureStorage storage;
  if (storage.Load(kDeviceIdKey, deviceId) && !deviceId.empty())
    return deviceId;

  deviceId = GenerateDeviceId();
  storage.Save(kDeviceIdKey, deviceId);
  return deviceId;
}

bool IdentityStore::HasUsername()
{
  std::string value;
  return settings::Get(std::string_view(kUsernameKey), value) && !value.empty();
}

std::string IdentityStore::GetUsername()
{
  std::string value;
  if (settings::Get(std::string_view(kUsernameKey), value))
    return value;
  return {};
}

bool IdentityStore::SetUsername(std::string_view username)
{
  std::string canonical(username);
  strings::AsciiToLower(canonical);
  if (!IsValidUsername(canonical))
    return false;
  settings::Set(std::string_view(kUsernameKey), canonical);
  return true;
}

bool IdentityStore::IsValidUsername(std::string_view username)
{
  if (username.size() < 3 || username.size() > 20)
    return false;

  for (char c : username)
  {
    // Enforce ASCII-only; allow only a–z, 0–9, underscore.
    if (static_cast<unsigned char>(c) > 0x7F)
      return false;
    if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'))
      return false;
  }
  return true;
}

std::string IdentityStore::GenerateDeviceId()
{
  std::array<unsigned char, 24> bytes{};
  std::random_device rd;
  std::generate(bytes.begin(), bytes.end(), [&rd]() { return static_cast<unsigned char>(rd()); });

  std::string raw(reinterpret_cast<char const *>(bytes.data()), bytes.size());
  auto const b64 = base64::Encode(std::string_view(raw.data(), raw.size()));
  return ToUrlSafeBase64(b64);
}