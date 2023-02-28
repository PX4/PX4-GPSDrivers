#include "unicore.h"
#include <cassert>
#include <cstdio>

void test_empty()
{
	const char str[] = " ";

	UnicoreParser unicore_parser;

	for (unsigned i = 0; i < sizeof(str); ++i) {
		auto result = unicore_parser.parseChar(str[i]);
		assert(result == UnicoreParser::Result::None);
	}
}

void test_garbage()
{
	const char str[] = "#GARBAGE,BLA";

	UnicoreParser unicore_parser;

	for (unsigned i = 0; i < sizeof(str); ++i) {
		auto result = unicore_parser.parseChar(str[i]);
		assert(result == UnicoreParser::Result::None);
	}
}

void test_too_long()
{
	const char str[] =
		"#UNIHEADINGA,89,GPS,FINE,2251,168052600,0,0,18,11;SOL_COMPUTED,"
		"NARROW_INT,0.3718,67.0255,-0.7974,0.0000,0.8065,3.3818,\"999\","
		"31,21,21,18,3,01,3,f3,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,ffffffff";

	UnicoreParser unicore_parser;

	for (unsigned i = 0; i < sizeof(str); ++i) {
		auto result = unicore_parser.parseChar(str[i]);
		assert(result == UnicoreParser::Result::None);
	}
}

void test_uniheadinga_wrong_crc()
{
	const char str[] =
		"#UNIHEADINGA,89,GPS,FINE,2251,168052600,0,0,18,11;SOL_COMPUTED,NARROW_INT,"
		"0.3718,67.0255,-0.7974,0.0000,0.8065,3.3818,\"999\",31,21,21,18,3,01,3,f3*"
		"ffffffff";

	UnicoreParser unicore_parser;

	for (unsigned i = 0; i < sizeof(str); ++i) {
		auto result = unicore_parser.parseChar(str[i]);

		if (result == UnicoreParser::Result::WrongCrc) {
			return;
		}
	}

	assert(false);
}


void test_uniheadinga()
{
	const char str[] =
		"#UNIHEADINGA,89,GPS,FINE,2251,168052600,0,0,18,11;SOL_COMPUTED,NARROW_INT,0.3718,67.0255,-0.7974,0.0000,0.8065,3.3818,\"999\",31,21,21,18,3,01,3,f3*ece5bb07";

	UnicoreParser unicore_parser;

	for (unsigned i = 0; i < sizeof(str); ++i) {
		auto result = unicore_parser.parseChar(str[i]);

		if (result == UnicoreParser::Result::GotHeading) {
			assert(unicore_parser.heading().heading_deg == 67.0255f);
			assert(unicore_parser.heading().baseline_m == 0.3718f);
			return;
		}
	}

	assert(false);
}

void test_unicore()
{
	test_empty();
	test_garbage();
	test_too_long();
	test_uniheadinga_wrong_crc();
	test_uniheadinga();
}

int main(int, char **)
{
	test_unicore();

	return 0;
}
