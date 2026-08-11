#pragma once

#include <Arduino.h>

/**
 * @file util.h
 * @brief 通信データの変換とジョイスティック入力処理用ユーティリティを提供する。
 */

/**
 * @brief 文字が16進数字か確認する。
 *
 * @param c 確認する文字。
 * @return 0〜9、A〜F、a〜fのいずれかの場合true。
 */
bool utilIsHexChar(char c);

/**
 * @brief 16進数字1文字を整数へ変換する。
 *
 * @param c 変換する文字。
 * @return 変換後の0〜15の値。16進数字でない場合は-1。
 */
int utilHexCharToInt(char c);

/**
 * @brief 文字列先頭の16進数字2文字を8ビット値へ変換する。
 *
 * @param text 変換対象の文字列。
 * @return 変換後の値。2文字未満または先頭2文字が不正な場合は0。
 */
uint8_t utilHexByteToUint8(const String& text);

/**
 * @brief 符号なし8ビット値を同じビット表現の符号付き8ビット値へ変換する。
 *
 * @param value 変換する8ビット値。
 * @return static_castによる符号付き8ビット変換結果。
 */
int8_t utilToInt8(uint8_t value);

/**
 * @brief 文字列から印字可能なASCII文字だけを抽出し、前後の空白を除去する。
 *
 * @param line 整形対象の文字列。
 * @return ASCII 0x20〜0x7Eだけで構成された整形後文字列。
 */
String utilSanitizeAsciiLine(const String& line);

/**
 * @brief 文字列に含まれる16進数字だけを順序を保って抽出する。
 *
 * @param text 抽出対象の文字列。
 * @return 抽出した16進数字の文字列。
 */
String utilCollectHexChars(const String& text);

/**
 * @brief 文字列の各バイトを大文字2桁の16進文字列へ変換する。
 *
 * @param text 変換対象の文字列。
 * @return 変換後の16進文字列。
 */
String utilTextToHex(const String& text);

/**
 * @brief 移動指令へデッドゾーンと範囲制限を適用して正規化する。
 *
 * @param value 正規化する移動指令値。
 * @return -1.0〜1.0の正規化値。絶対値がデッドゾーン未満の場合は0.0。
 */
float utilCommandToFloat(int value);
