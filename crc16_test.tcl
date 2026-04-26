# crc16_test.tcl
# PJ 2026-04-10
# $ tclsh crc16_test.tcl
# 15159
#
package require crc16
set data "\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f"
puts [::crc::crc-ccitt -seed 0xffff -- $data]
