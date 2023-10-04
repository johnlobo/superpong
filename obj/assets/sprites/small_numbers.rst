                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 4.2.0 #13081 (Mac OS X ppc)
                                      4 ;--------------------------------------------------------
                                      5 	.module small_numbers
                                      6 	.optsdcc -mz80
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _s_small_numbers_09
                                     12 	.globl _s_small_numbers_08
                                     13 	.globl _s_small_numbers_07
                                     14 	.globl _s_small_numbers_06
                                     15 	.globl _s_small_numbers_05
                                     16 	.globl _s_small_numbers_04
                                     17 	.globl _s_small_numbers_03
                                     18 	.globl _s_small_numbers_02
                                     19 	.globl _s_small_numbers_01
                                     20 	.globl _s_small_numbers_00
                                     21 ;--------------------------------------------------------
                                     22 ; special function registers
                                     23 ;--------------------------------------------------------
                                     24 ;--------------------------------------------------------
                                     25 ; ram data
                                     26 ;--------------------------------------------------------
                                     27 	.area _DATA
                                     28 ;--------------------------------------------------------
                                     29 ; ram data
                                     30 ;--------------------------------------------------------
                                     31 	.area _INITIALIZED
                                     32 ;--------------------------------------------------------
                                     33 ; absolute external ram data
                                     34 ;--------------------------------------------------------
                                     35 	.area _DABS (ABS)
                                     36 ;--------------------------------------------------------
                                     37 ; global & static initialisations
                                     38 ;--------------------------------------------------------
                                     39 	.area _HOME
                                     40 	.area _GSINIT
                                     41 	.area _GSFINAL
                                     42 	.area _GSINIT
                                     43 ;--------------------------------------------------------
                                     44 ; Home
                                     45 ;--------------------------------------------------------
                                     46 	.area _HOME
                                     47 	.area _HOME
                                     48 ;--------------------------------------------------------
                                     49 ; code
                                     50 ;--------------------------------------------------------
                                     51 	.area _CODE
                                     52 	.area _CODE
      000200                         53 _s_small_numbers_00:
      000200 FF                      54 	.db #0xff	; 255
      000201 AA                      55 	.db #0xaa	; 170
      000202 AA                      56 	.db #0xaa	; 170
      000203 AA                      57 	.db #0xaa	; 170
      000204 AA                      58 	.db #0xaa	; 170
      000205 AA                      59 	.db #0xaa	; 170
      000206 AA                      60 	.db #0xaa	; 170
      000207 AA                      61 	.db #0xaa	; 170
      000208 FF                      62 	.db #0xff	; 255
      000209 AA                      63 	.db #0xaa	; 170
      00020A                         64 _s_small_numbers_01:
      00020A 55                      65 	.db #0x55	; 85	'U'
      00020B AA                      66 	.db #0xaa	; 170
      00020C 00                      67 	.db #0x00	; 0
      00020D AA                      68 	.db #0xaa	; 170
      00020E 00                      69 	.db #0x00	; 0
      00020F AA                      70 	.db #0xaa	; 170
      000210 00                      71 	.db #0x00	; 0
      000211 AA                      72 	.db #0xaa	; 170
      000212 00                      73 	.db #0x00	; 0
      000213 AA                      74 	.db #0xaa	; 170
      000214                         75 _s_small_numbers_02:
      000214 FF                      76 	.db #0xff	; 255
      000215 AA                      77 	.db #0xaa	; 170
      000216 00                      78 	.db #0x00	; 0
      000217 AA                      79 	.db #0xaa	; 170
      000218 FF                      80 	.db #0xff	; 255
      000219 AA                      81 	.db #0xaa	; 170
      00021A AA                      82 	.db #0xaa	; 170
      00021B 00                      83 	.db #0x00	; 0
      00021C FF                      84 	.db #0xff	; 255
      00021D AA                      85 	.db #0xaa	; 170
      00021E                         86 _s_small_numbers_03:
      00021E FF                      87 	.db #0xff	; 255
      00021F AA                      88 	.db #0xaa	; 170
      000220 00                      89 	.db #0x00	; 0
      000221 AA                      90 	.db #0xaa	; 170
      000222 55                      91 	.db #0x55	; 85	'U'
      000223 AA                      92 	.db #0xaa	; 170
      000224 00                      93 	.db #0x00	; 0
      000225 AA                      94 	.db #0xaa	; 170
      000226 FF                      95 	.db #0xff	; 255
      000227 AA                      96 	.db #0xaa	; 170
      000228                         97 _s_small_numbers_04:
      000228 AA                      98 	.db #0xaa	; 170
      000229 AA                      99 	.db #0xaa	; 170
      00022A AA                     100 	.db #0xaa	; 170
      00022B AA                     101 	.db #0xaa	; 170
      00022C FF                     102 	.db #0xff	; 255
      00022D AA                     103 	.db #0xaa	; 170
      00022E 00                     104 	.db #0x00	; 0
      00022F AA                     105 	.db #0xaa	; 170
      000230 00                     106 	.db #0x00	; 0
      000231 AA                     107 	.db #0xaa	; 170
      000232                        108 _s_small_numbers_05:
      000232 FF                     109 	.db #0xff	; 255
      000233 AA                     110 	.db #0xaa	; 170
      000234 AA                     111 	.db #0xaa	; 170
      000235 00                     112 	.db #0x00	; 0
      000236 FF                     113 	.db #0xff	; 255
      000237 AA                     114 	.db #0xaa	; 170
      000238 00                     115 	.db #0x00	; 0
      000239 AA                     116 	.db #0xaa	; 170
      00023A FF                     117 	.db #0xff	; 255
      00023B AA                     118 	.db #0xaa	; 170
      00023C                        119 _s_small_numbers_06:
      00023C FF                     120 	.db #0xff	; 255
      00023D AA                     121 	.db #0xaa	; 170
      00023E AA                     122 	.db #0xaa	; 170
      00023F 00                     123 	.db #0x00	; 0
      000240 FF                     124 	.db #0xff	; 255
      000241 AA                     125 	.db #0xaa	; 170
      000242 AA                     126 	.db #0xaa	; 170
      000243 AA                     127 	.db #0xaa	; 170
      000244 FF                     128 	.db #0xff	; 255
      000245 AA                     129 	.db #0xaa	; 170
      000246                        130 _s_small_numbers_07:
      000246 FF                     131 	.db #0xff	; 255
      000247 AA                     132 	.db #0xaa	; 170
      000248 00                     133 	.db #0x00	; 0
      000249 AA                     134 	.db #0xaa	; 170
      00024A 00                     135 	.db #0x00	; 0
      00024B AA                     136 	.db #0xaa	; 170
      00024C 00                     137 	.db #0x00	; 0
      00024D AA                     138 	.db #0xaa	; 170
      00024E 00                     139 	.db #0x00	; 0
      00024F AA                     140 	.db #0xaa	; 170
      000250                        141 _s_small_numbers_08:
      000250 FF                     142 	.db #0xff	; 255
      000251 AA                     143 	.db #0xaa	; 170
      000252 AA                     144 	.db #0xaa	; 170
      000253 AA                     145 	.db #0xaa	; 170
      000254 FF                     146 	.db #0xff	; 255
      000255 AA                     147 	.db #0xaa	; 170
      000256 AA                     148 	.db #0xaa	; 170
      000257 AA                     149 	.db #0xaa	; 170
      000258 FF                     150 	.db #0xff	; 255
      000259 AA                     151 	.db #0xaa	; 170
      00025A                        152 _s_small_numbers_09:
      00025A FF                     153 	.db #0xff	; 255
      00025B AA                     154 	.db #0xaa	; 170
      00025C AA                     155 	.db #0xaa	; 170
      00025D AA                     156 	.db #0xaa	; 170
      00025E FF                     157 	.db #0xff	; 255
      00025F AA                     158 	.db #0xaa	; 170
      000260 00                     159 	.db #0x00	; 0
      000261 AA                     160 	.db #0xaa	; 170
      000262 00                     161 	.db #0x00	; 0
      000263 AA                     162 	.db #0xaa	; 170
                                    163 	.area _INITIALIZER
                                    164 	.area _CABS (ABS)
