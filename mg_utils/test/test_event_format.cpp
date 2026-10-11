#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "mg_utils/event_format.hpp"

using mg_utils::FormatEvent;
using mg_utils::QuoteIfNeeded;

TEST(EventFormat, SimpleEvent)
{
  EXPECT_EQ(
    FormatEvent("sequencer.state_changed", {{"from", "IDLE"}, {"to", "NAVIGATING"}, {"index", 3}}),
    "evt=sequencer.state_changed from=IDLE to=NAVIGATING index=3");
}

TEST(EventFormat, NoFields)
{
  EXPECT_EQ(FormatEvent("node.started"), "evt=node.started");
}

TEST(EventFormat, ValueTypes)
{
  EXPECT_EQ(
    FormatEvent("a.b", {{"t", true}, {"f", false}, {"x", 1.5}, {"y", 0.1 + 0.2}, {"n", -7}}),
    "evt=a.b t=true f=false x=1.5 y=0.3 n=-7");
}

TEST(EventFormat, QuotesAndEscapes)
{
  EXPECT_EQ(QuoteIfNeeded(""), "\"\"");
  EXPECT_EQ(QuoteIfNeeded("read timeout"), "\"read timeout\"");
  EXPECT_EQ(QuoteIfNeeded("a=b"), "\"a=b\"");
  EXPECT_EQ(QuoteIfNeeded("say \"hi\""), "\"say \\\"hi\\\"\"");
  EXPECT_EQ(QuoteIfNeeded("back\\slash"), "\"back\\\\slash\"");
  EXPECT_EQ(QuoteIfNeeded("line1\nline2"), "\"line1\\nline2\"");
  EXPECT_EQ(QuoteIfNeeded("plain"), "plain");
}

TEST(EventFormat, StringFieldIsQuoted)
{
  const std::string reason = "read timeout";
  EXPECT_EQ(
    FormatEvent("driver.disconnected", {{"reason", reason}}),
    "evt=driver.disconnected reason=\"read timeout\"");
}

TEST(EventFormat, InvalidName)
{
  for (const char * name : {"started", "Node.started", "node.Started", "node.", ".started",
      "a.b.c"})
  {
    EXPECT_THROW(FormatEvent(name), std::invalid_argument) << name;
  }
}

TEST(EventFormat, InvalidKey)
{
  EXPECT_THROW(FormatEvent("a.b", {{"Index", 1}}), std::invalid_argument);
  EXPECT_THROW(FormatEvent("a.b", {{"a-b", 1}}), std::invalid_argument);
  EXPECT_THROW(FormatEvent("a.b", {{"evt", 1}}), std::invalid_argument);
}
