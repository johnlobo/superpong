                              1 ;--------------------------------------------------------
                              2 ; File Created by SDCC : free open source ANSI-C Compiler
                              3 ; Version 3.6.8 #9946 (Linux)
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
   0200                      53 _s_small_numbers_00:
   0200 FF                   54 	.db #0xff	; 255
   0201 AA                   55 	.db #0xaa	; 170
   0202 AA                   56 	.db #0xaa	; 170
   0203 AA                   57 	.db #0xaa	; 170
   0204 AA                   58 	.db #0xaa	; 170
   0205 AA                   59 	.db #0xaa	; 170
   0206 AA                   60 	.db #0xaa	; 170
   0207 AA                   61 	.db #0xaa	; 170
   0208 FF                   62 	.db #0xff	; 255
   0209 AA                   63 	.db #0xaa	; 170
   020A                      64 _s_small_numbers_01:
   020A 55                   65 	.db #0x55	; 85	'U'
   020B AA                   66 	.db #0xaa	; 170
   020C 00                   67 	.db #0x00	; 0
   020D AA                   68 	.db #0xaa	; 170
   020E 00                   69 	.db #0x00	; 0
   020F AA                   70 	.db #0xaa	; 170
   0210 00                   71 	.db #0x00	; 0
   0211 AA                   72 	.db #0xaa	; 170
   0212 00                   73 	.db #0x00	; 0
   0213 AA                   74 	.db #0xaa	; 170
   0214                      75 _s_small_numbers_02:
   0214 FF                   76 	.db #0xff	; 255
   0215 AA                   77 	.db #0xaa	; 170
   0216 00                   78 	.db #0x00	; 0
   0217 AA                   79 	.db #0xaa	; 170
   0218 FF                   80 	.db #0xff	; 255
   0219 AA                   81 	.db #0xaa	; 170
   021A AA                   82 	.db #0xaa	; 170
   021B 00                   83 	.db #0x00	; 0
   021C FF                   84 	.db #0xff	; 255
   021D AA                   85 	.db #0xaa	; 170
   021E                      86 _s_small_numbers_03:
   021E FF                   87 	.db #0xff	; 255
   021F AA                   88 	.db #0xaa	; 170
   0220 00                   89 	.db #0x00	; 0
   0221 AA                   90 	.db #0xaa	; 170
   0222 55                   91 	.db #0x55	; 85	'U'
   0223 AA                   92 	.db #0xaa	; 170
   0224 00                   93 	.db #0x00	; 0
   0225 AA                   94 	.db #0xaa	; 170
   0226 FF                   95 	.db #0xff	; 255
   0227 AA                   96 	.db #0xaa	; 170
   0228                      97 _s_small_numbers_04:
   0228 AA                   98 	.db #0xaa	; 170
   0229 AA                   99 	.db #0xaa	; 170
   022A AA                  100 	.db #0xaa	; 170
   022B AA                  101 	.db #0xaa	; 170
   022C FF                  102 	.db #0xff	; 255
   022D AA                  103 	.db #0xaa	; 170
   022E 00                  104 	.db #0x00	; 0
   022F AA                  105 	.db #0xaa	; 170
   0230 00                  106 	.db #0x00	; 0
   0231 AA                  107 	.db #0xaa	; 170
   0232                     108 _s_small_numbers_05:
   0232 FF                  109 	.db #0xff	; 255
   0233 AA                  110 	.db #0xaa	; 170
   0234 AA                  111 	.db #0xaa	; 170
   0235 00                  112 	.db #0x00	; 0
   0236 FF                  113 	.db #0xff	; 255
   0237 AA                  114 	.db #0xaa	; 170
   0238 00                  115 	.db #0x00	; 0
   0239 AA                  116 	.db #0xaa	; 170
   023A FF                  117 	.db #0xff	; 255
   023B AA                  118 	.db #0xaa	; 170
   023C                     119 _s_small_numbers_06:
   023C FF                  120 	.db #0xff	; 255
   023D AA                  121 	.db #0xaa	; 170
   023E AA                  122 	.db #0xaa	; 170
   023F 00                  123 	.db #0x00	; 0
   0240 FF                  124 	.db #0xff	; 255
   0241 AA                  125 	.db #0xaa	; 170
   0242 AA                  126 	.db #0xaa	; 170
   0243 AA                  127 	.db #0xaa	; 170
   0244 FF                  128 	.db #0xff	; 255
   0245 AA                  129 	.db #0xaa	; 170
   0246                     130 _s_small_numbers_07:
   0246 FF                  131 	.db #0xff	; 255
   0247 AA                  132 	.db #0xaa	; 170
   0248 00                  133 	.db #0x00	; 0
   0249 AA                  134 	.db #0xaa	; 170
   024A 00                  135 	.db #0x00	; 0
   024B AA                  136 	.db #0xaa	; 170
   024C 00                  137 	.db #0x00	; 0
   024D AA                  138 	.db #0xaa	; 170
   024E 00                  139 	.db #0x00	; 0
   024F AA                  140 	.db #0xaa	; 170
   0250                     141 _s_small_numbers_08:
   0250 FF                  142 	.db #0xff	; 255
   0251 AA                  143 	.db #0xaa	; 170
   0252 AA                  144 	.db #0xaa	; 170
   0253 AA                  145 	.db #0xaa	; 170
   0254 FF                  146 	.db #0xff	; 255
   0255 AA                  147 	.db #0xaa	; 170
   0256 AA                  148 	.db #0xaa	; 170
   0257 AA                  149 	.db #0xaa	; 170
   0258 FF                  150 	.db #0xff	; 255
   0259 AA                  151 	.db #0xaa	; 170
   025A                     152 _s_small_numbers_09:
   025A FF                  153 	.db #0xff	; 255
   025B AA                  154 	.db #0xaa	; 170
   025C AA                  155 	.db #0xaa	; 170
   025D AA                  156 	.db #0xaa	; 170
   025E FF                  157 	.db #0xff	; 255
   025F AA                  158 	.db #0xaa	; 170
   0260 00                  159 	.db #0x00	; 0
   0261 AA                  160 	.db #0xaa	; 170
   0262 00                  161 	.db #0x00	; 0
   0263 AA                  162 	.db #0xaa	; 170
                            163 	.area _INITIALIZER
                            164 	.area _CABS (ABS)
