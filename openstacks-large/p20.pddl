(define (problem os-sequencedstrips-p290_2)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210 n211 n212 n213 n214 n215 n216 n217 n218 n219 n220 n221 n222 n223 n224 n225 n226 n227 n228 n229 n230 n231 n232 n233 n234 n235 n236 n237 n238 n239 n240 n241 n242 n243 n244 n245 n246 n247 n248 n249 n250 n251 n252 n253 n254 n255 n256 n257 n258 n259 n260 n261 n262 n263 n264 n265 n266 n267 n268 n269 n270 n271 n272 n273 n274 n275 n276 n277 n278 n279 n280 n281 n282 n283 n284 n285 n286 n287 n288 n289 n290  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) (next-count n210 n211) (next-count n211 n212) (next-count n212 n213) (next-count n213 n214) (next-count n214 n215) (next-count n215 n216) (next-count n216 n217) (next-count n217 n218) (next-count n218 n219) (next-count n219 n220) (next-count n220 n221) (next-count n221 n222) (next-count n222 n223) (next-count n223 n224) (next-count n224 n225) (next-count n225 n226) (next-count n226 n227) (next-count n227 n228) (next-count n228 n229) (next-count n229 n230) (next-count n230 n231) (next-count n231 n232) (next-count n232 n233) (next-count n233 n234) (next-count n234 n235) (next-count n235 n236) (next-count n236 n237) (next-count n237 n238) (next-count n238 n239) (next-count n239 n240) (next-count n240 n241) (next-count n241 n242) (next-count n242 n243) (next-count n243 n244) (next-count n244 n245) (next-count n245 n246) (next-count n246 n247) (next-count n247 n248) (next-count n248 n249) (next-count n249 n250) (next-count n250 n251) (next-count n251 n252) (next-count n252 n253) (next-count n253 n254) (next-count n254 n255) (next-count n255 n256) (next-count n256 n257) (next-count n257 n258) (next-count n258 n259) (next-count n259 n260) (next-count n260 n261) (next-count n261 n262) (next-count n262 n263) (next-count n263 n264) (next-count n264 n265) (next-count n265 n266) (next-count n266 n267) (next-count n267 n268) (next-count n268 n269) (next-count n269 n270) (next-count n270 n271) (next-count n271 n272) (next-count n272 n273) (next-count n273 n274) (next-count n274 n275) (next-count n275 n276) (next-count n276 n277) (next-count n277 n278) (next-count n278 n279) (next-count n279 n280) (next-count n280 n281) (next-count n281 n282) (next-count n282 n283) (next-count n283 n284) (next-count n284 n285) (next-count n285 n286) (next-count n286 n287) (next-count n287 n288) (next-count n288 n289) (next-count n289 n290) 
(stacks-avail n0)

(waiting o1)
(includes o1 p1)(includes o1 p49)(includes o1 p169)(includes o1 p207)(includes o1 p275)

(waiting o2)
(includes o2 p32)(includes o2 p186)(includes o2 p203)(includes o2 p205)(includes o2 p235)(includes o2 p240)(includes o2 p264)(includes o2 p272)

(waiting o3)
(includes o3 p43)(includes o3 p57)(includes o3 p205)(includes o3 p207)

(waiting o4)
(includes o4 p29)(includes o4 p74)(includes o4 p105)(includes o4 p130)(includes o4 p189)(includes o4 p207)(includes o4 p209)(includes o4 p238)(includes o4 p256)

(waiting o5)
(includes o5 p66)(includes o5 p67)(includes o5 p82)(includes o5 p112)(includes o5 p190)

(waiting o6)
(includes o6 p8)(includes o6 p21)(includes o6 p48)(includes o6 p122)(includes o6 p146)(includes o6 p156)(includes o6 p158)(includes o6 p162)(includes o6 p268)(includes o6 p282)

(waiting o7)
(includes o7 p40)(includes o7 p64)(includes o7 p81)(includes o7 p83)(includes o7 p107)(includes o7 p171)(includes o7 p176)(includes o7 p280)

(waiting o8)
(includes o8 p13)(includes o8 p31)(includes o8 p35)(includes o8 p79)(includes o8 p116)(includes o8 p136)(includes o8 p164)(includes o8 p230)(includes o8 p234)(includes o8 p250)(includes o8 p277)(includes o8 p288)

(waiting o9)
(includes o9 p17)(includes o9 p136)(includes o9 p171)(includes o9 p173)(includes o9 p263)

(waiting o10)
(includes o10 p4)(includes o10 p33)(includes o10 p42)(includes o10 p44)(includes o10 p53)(includes o10 p160)(includes o10 p264)(includes o10 p268)(includes o10 p272)

(waiting o11)
(includes o11 p16)(includes o11 p81)(includes o11 p113)(includes o11 p124)(includes o11 p193)

(waiting o12)
(includes o12 p19)(includes o12 p68)(includes o12 p139)(includes o12 p159)(includes o12 p181)(includes o12 p250)

(waiting o13)
(includes o13 p7)(includes o13 p50)(includes o13 p53)(includes o13 p105)(includes o13 p134)(includes o13 p162)(includes o13 p175)(includes o13 p205)(includes o13 p218)(includes o13 p264)

(waiting o14)
(includes o14 p48)(includes o14 p153)(includes o14 p190)(includes o14 p220)(includes o14 p239)(includes o14 p242)(includes o14 p251)

(waiting o15)
(includes o15 p12)(includes o15 p22)(includes o15 p27)(includes o15 p32)(includes o15 p82)(includes o15 p202)(includes o15 p235)

(waiting o16)
(includes o16 p6)(includes o16 p104)(includes o16 p166)(includes o16 p188)(includes o16 p212)(includes o16 p213)(includes o16 p284)

(waiting o17)
(includes o17 p33)(includes o17 p96)(includes o17 p151)(includes o17 p176)(includes o17 p221)(includes o17 p236)(includes o17 p269)(includes o17 p282)

(waiting o18)
(includes o18 p55)(includes o18 p79)(includes o18 p162)(includes o18 p186)(includes o18 p195)(includes o18 p260)

(waiting o19)
(includes o19 p22)(includes o19 p43)(includes o19 p125)(includes o19 p154)(includes o19 p166)(includes o19 p237)

(waiting o20)
(includes o20 p9)(includes o20 p55)(includes o20 p84)(includes o20 p113)(includes o20 p182)(includes o20 p207)(includes o20 p234)(includes o20 p273)(includes o20 p276)

(waiting o21)
(includes o21 p43)(includes o21 p80)(includes o21 p92)(includes o21 p122)(includes o21 p132)

(waiting o22)
(includes o22 p9)(includes o22 p17)(includes o22 p19)(includes o22 p48)(includes o22 p55)(includes o22 p102)(includes o22 p126)(includes o22 p136)(includes o22 p165)(includes o22 p245)

(waiting o23)
(includes o23 p17)(includes o23 p31)(includes o23 p101)(includes o23 p138)(includes o23 p151)(includes o23 p195)(includes o23 p218)

(waiting o24)
(includes o24 p27)(includes o24 p86)(includes o24 p168)(includes o24 p200)(includes o24 p283)

(waiting o25)
(includes o25 p35)(includes o25 p53)(includes o25 p71)(includes o25 p103)(includes o25 p112)(includes o25 p121)(includes o25 p190)(includes o25 p220)(includes o25 p225)(includes o25 p230)(includes o25 p268)(includes o25 p277)(includes o25 p289)

(waiting o26)
(includes o26 p99)(includes o26 p143)(includes o26 p243)

(waiting o27)
(includes o27 p32)(includes o27 p52)(includes o27 p79)(includes o27 p96)(includes o27 p104)(includes o27 p277)

(waiting o28)
(includes o28 p47)(includes o28 p128)(includes o28 p152)(includes o28 p188)(includes o28 p198)(includes o28 p215)

(waiting o29)
(includes o29 p9)(includes o29 p130)(includes o29 p155)(includes o29 p165)(includes o29 p217)(includes o29 p222)(includes o29 p250)

(waiting o30)
(includes o30 p115)(includes o30 p121)(includes o30 p126)(includes o30 p151)

(waiting o31)
(includes o31 p23)(includes o31 p99)(includes o31 p140)(includes o31 p227)(includes o31 p259)

(waiting o32)
(includes o32 p49)(includes o32 p80)(includes o32 p95)(includes o32 p122)

(waiting o33)
(includes o33 p54)(includes o33 p112)(includes o33 p123)(includes o33 p142)(includes o33 p247)(includes o33 p274)

(waiting o34)
(includes o34 p18)(includes o34 p39)(includes o34 p52)(includes o34 p58)(includes o34 p134)(includes o34 p145)(includes o34 p161)(includes o34 p213)(includes o34 p254)(includes o34 p260)

(waiting o35)
(includes o35 p2)(includes o35 p3)(includes o35 p67)(includes o35 p69)(includes o35 p74)(includes o35 p80)(includes o35 p125)(includes o35 p144)(includes o35 p224)(includes o35 p264)

(waiting o36)
(includes o36 p5)(includes o36 p10)(includes o36 p26)(includes o36 p32)(includes o36 p86)(includes o36 p95)(includes o36 p120)(includes o36 p151)(includes o36 p193)

(waiting o37)
(includes o37 p11)(includes o37 p16)(includes o37 p58)(includes o37 p122)(includes o37 p137)(includes o37 p162)(includes o37 p181)(includes o37 p239)

(waiting o38)
(includes o38 p107)(includes o38 p119)(includes o38 p150)(includes o38 p157)(includes o38 p167)(includes o38 p184)

(waiting o39)
(includes o39 p24)(includes o39 p110)(includes o39 p134)(includes o39 p145)(includes o39 p151)(includes o39 p193)(includes o39 p228)(includes o39 p243)(includes o39 p276)

(waiting o40)
(includes o40 p10)(includes o40 p11)(includes o40 p50)(includes o40 p125)(includes o40 p212)

(waiting o41)
(includes o41 p43)(includes o41 p105)(includes o41 p136)(includes o41 p147)(includes o41 p158)(includes o41 p187)(includes o41 p228)(includes o41 p277)

(waiting o42)
(includes o42 p46)(includes o42 p62)(includes o42 p128)(includes o42 p168)(includes o42 p178)(includes o42 p192)(includes o42 p225)

(waiting o43)
(includes o43 p34)(includes o43 p139)(includes o43 p225)(includes o43 p243)(includes o43 p281)

(waiting o44)
(includes o44 p24)(includes o44 p88)(includes o44 p128)(includes o44 p177)(includes o44 p247)(includes o44 p275)

(waiting o45)
(includes o45 p3)(includes o45 p14)(includes o45 p26)(includes o45 p210)(includes o45 p224)

(waiting o46)
(includes o46 p122)(includes o46 p218)(includes o46 p280)

(waiting o47)
(includes o47 p47)(includes o47 p126)(includes o47 p129)(includes o47 p222)(includes o47 p249)(includes o47 p254)

(waiting o48)
(includes o48 p93)(includes o48 p101)(includes o48 p108)(includes o48 p123)(includes o48 p130)(includes o48 p148)(includes o48 p186)(includes o48 p230)(includes o48 p270)(includes o48 p280)

(waiting o49)
(includes o49 p45)(includes o49 p94)(includes o49 p132)

(waiting o50)
(includes o50 p165)(includes o50 p229)(includes o50 p270)

(waiting o51)
(includes o51 p22)(includes o51 p164)(includes o51 p252)(includes o51 p261)

(waiting o52)
(includes o52 p18)(includes o52 p46)(includes o52 p118)(includes o52 p203)(includes o52 p215)(includes o52 p248)(includes o52 p260)

(waiting o53)
(includes o53 p5)(includes o53 p25)(includes o53 p98)(includes o53 p100)(includes o53 p135)(includes o53 p146)(includes o53 p158)(includes o53 p171)(includes o53 p175)(includes o53 p228)(includes o53 p260)

(waiting o54)
(includes o54 p22)(includes o54 p78)(includes o54 p118)(includes o54 p242)(includes o54 p251)

(waiting o55)
(includes o55 p80)(includes o55 p145)(includes o55 p155)(includes o55 p196)(includes o55 p214)(includes o55 p219)(includes o55 p254)

(waiting o56)
(includes o56 p11)(includes o56 p146)(includes o56 p153)(includes o56 p157)(includes o56 p231)

(waiting o57)
(includes o57 p66)(includes o57 p150)(includes o57 p227)(includes o57 p284)

(waiting o58)
(includes o58 p9)(includes o58 p114)(includes o58 p208)(includes o58 p221)(includes o58 p265)

(waiting o59)
(includes o59 p35)(includes o59 p128)(includes o59 p242)

(waiting o60)
(includes o60 p9)(includes o60 p16)(includes o60 p27)(includes o60 p133)(includes o60 p198)(includes o60 p223)(includes o60 p276)

(waiting o61)
(includes o61 p18)(includes o61 p48)(includes o61 p49)(includes o61 p50)(includes o61 p57)(includes o61 p131)

(waiting o62)
(includes o62 p34)(includes o62 p85)(includes o62 p132)(includes o62 p166)(includes o62 p172)(includes o62 p201)(includes o62 p275)

(waiting o63)
(includes o63 p4)(includes o63 p15)(includes o63 p45)(includes o63 p135)

(waiting o64)
(includes o64 p3)(includes o64 p101)(includes o64 p140)(includes o64 p157)(includes o64 p182)(includes o64 p208)(includes o64 p231)(includes o64 p246)(includes o64 p260)(includes o64 p263)(includes o64 p270)

(waiting o65)
(includes o65 p6)(includes o65 p32)(includes o65 p47)(includes o65 p48)(includes o65 p81)(includes o65 p83)(includes o65 p84)(includes o65 p142)

(waiting o66)
(includes o66 p195)

(waiting o67)
(includes o67 p143)(includes o67 p212)

(waiting o68)
(includes o68 p111)(includes o68 p209)(includes o68 p269)(includes o68 p284)

(waiting o69)
(includes o69 p11)(includes o69 p33)(includes o69 p109)(includes o69 p180)(includes o69 p214)(includes o69 p228)(includes o69 p235)(includes o69 p253)(includes o69 p262)

(waiting o70)
(includes o70 p42)(includes o70 p51)(includes o70 p172)(includes o70 p239)

(waiting o71)
(includes o71 p51)(includes o71 p67)(includes o71 p113)(includes o71 p122)(includes o71 p168)(includes o71 p201)(includes o71 p202)(includes o71 p223)(includes o71 p225)(includes o71 p245)

(waiting o72)
(includes o72 p75)(includes o72 p127)(includes o72 p132)(includes o72 p188)(includes o72 p214)(includes o72 p245)(includes o72 p262)

(waiting o73)
(includes o73 p8)(includes o73 p54)(includes o73 p168)(includes o73 p248)(includes o73 p289)

(waiting o74)
(includes o74 p18)(includes o74 p158)(includes o74 p169)(includes o74 p245)

(waiting o75)
(includes o75 p4)(includes o75 p23)(includes o75 p237)(includes o75 p240)(includes o75 p259)(includes o75 p278)

(waiting o76)
(includes o76 p47)(includes o76 p95)(includes o76 p134)(includes o76 p167)

(waiting o77)
(includes o77 p4)(includes o77 p10)(includes o77 p93)(includes o77 p163)(includes o77 p212)(includes o77 p219)(includes o77 p238)(includes o77 p251)(includes o77 p259)(includes o77 p289)

(waiting o78)
(includes o78 p86)(includes o78 p143)(includes o78 p176)(includes o78 p220)(includes o78 p288)

(waiting o79)
(includes o79 p7)(includes o79 p16)(includes o79 p59)(includes o79 p70)(includes o79 p77)(includes o79 p90)(includes o79 p148)(includes o79 p151)(includes o79 p216)(includes o79 p281)(includes o79 p284)

(waiting o80)
(includes o80 p12)(includes o80 p80)(includes o80 p102)(includes o80 p166)(includes o80 p202)(includes o80 p236)(includes o80 p266)

(waiting o81)
(includes o81 p16)(includes o81 p32)(includes o81 p47)(includes o81 p110)(includes o81 p169)(includes o81 p267)

(waiting o82)
(includes o82 p15)(includes o82 p27)(includes o82 p54)(includes o82 p73)(includes o82 p179)(includes o82 p233)(includes o82 p287)(includes o82 p289)

(waiting o83)
(includes o83 p2)(includes o83 p63)(includes o83 p133)(includes o83 p146)(includes o83 p277)

(waiting o84)
(includes o84 p20)(includes o84 p65)(includes o84 p68)(includes o84 p88)(includes o84 p103)(includes o84 p155)(includes o84 p200)(includes o84 p247)

(waiting o85)
(includes o85 p2)(includes o85 p10)(includes o85 p25)(includes o85 p43)(includes o85 p144)(includes o85 p150)(includes o85 p180)(includes o85 p181)(includes o85 p218)

(waiting o86)
(includes o86 p62)(includes o86 p76)(includes o86 p121)(includes o86 p135)(includes o86 p138)(includes o86 p147)(includes o86 p190)(includes o86 p199)(includes o86 p208)(includes o86 p213)(includes o86 p226)(includes o86 p241)(includes o86 p243)(includes o86 p283)

(waiting o87)
(includes o87 p2)(includes o87 p10)(includes o87 p55)(includes o87 p96)(includes o87 p109)(includes o87 p285)

(waiting o88)
(includes o88 p78)(includes o88 p141)(includes o88 p163)(includes o88 p232)(includes o88 p280)

(waiting o89)
(includes o89 p3)(includes o89 p34)(includes o89 p136)(includes o89 p159)(includes o89 p237)(includes o89 p287)

(waiting o90)
(includes o90 p27)(includes o90 p69)(includes o90 p128)(includes o90 p151)(includes o90 p157)(includes o90 p208)(includes o90 p255)(includes o90 p290)

(waiting o91)
(includes o91 p25)(includes o91 p43)(includes o91 p190)(includes o91 p263)(includes o91 p273)

(waiting o92)
(includes o92 p111)(includes o92 p188)(includes o92 p239)(includes o92 p257)

(waiting o93)
(includes o93 p20)(includes o93 p74)(includes o93 p97)(includes o93 p119)(includes o93 p127)(includes o93 p161)(includes o93 p289)

(waiting o94)
(includes o94 p167)(includes o94 p169)(includes o94 p210)(includes o94 p279)

(waiting o95)
(includes o95 p29)(includes o95 p62)(includes o95 p86)(includes o95 p255)(includes o95 p287)

(waiting o96)
(includes o96 p1)(includes o96 p59)(includes o96 p78)

(waiting o97)
(includes o97 p38)(includes o97 p82)(includes o97 p94)(includes o97 p97)(includes o97 p106)(includes o97 p159)(includes o97 p164)(includes o97 p231)

(waiting o98)
(includes o98 p28)(includes o98 p38)(includes o98 p156)(includes o98 p237)(includes o98 p250)(includes o98 p268)

(waiting o99)
(includes o99 p21)(includes o99 p56)(includes o99 p58)(includes o99 p178)(includes o99 p241)(includes o99 p285)

(waiting o100)
(includes o100 p60)(includes o100 p82)(includes o100 p91)(includes o100 p96)(includes o100 p152)(includes o100 p240)(includes o100 p244)(includes o100 p247)

(waiting o101)
(includes o101 p141)(includes o101 p153)(includes o101 p206)(includes o101 p212)

(waiting o102)
(includes o102 p16)(includes o102 p170)(includes o102 p256)

(waiting o103)
(includes o103 p1)(includes o103 p33)(includes o103 p52)(includes o103 p187)(includes o103 p256)(includes o103 p258)(includes o103 p283)

(waiting o104)
(includes o104 p31)(includes o104 p50)(includes o104 p100)(includes o104 p113)(includes o104 p215)(includes o104 p257)(includes o104 p274)

(waiting o105)
(includes o105 p63)(includes o105 p74)(includes o105 p100)(includes o105 p143)(includes o105 p249)

(waiting o106)
(includes o106 p52)(includes o106 p62)(includes o106 p87)(includes o106 p89)(includes o106 p151)(includes o106 p166)(includes o106 p183)(includes o106 p192)(includes o106 p247)(includes o106 p248)(includes o106 p257)

(waiting o107)
(includes o107 p25)(includes o107 p36)(includes o107 p96)(includes o107 p97)(includes o107 p122)(includes o107 p127)(includes o107 p218)(includes o107 p227)(includes o107 p260)

(waiting o108)
(includes o108 p9)(includes o108 p69)(includes o108 p109)(includes o108 p170)(includes o108 p196)(includes o108 p200)(includes o108 p208)(includes o108 p244)(includes o108 p252)(includes o108 p264)

(waiting o109)
(includes o109 p16)(includes o109 p24)(includes o109 p44)(includes o109 p57)(includes o109 p81)(includes o109 p82)(includes o109 p96)(includes o109 p219)(includes o109 p279)

(waiting o110)
(includes o110 p6)(includes o110 p9)(includes o110 p80)(includes o110 p101)(includes o110 p152)(includes o110 p185)(includes o110 p274)

(waiting o111)
(includes o111 p90)(includes o111 p228)(includes o111 p230)(includes o111 p289)

(waiting o112)
(includes o112 p32)(includes o112 p87)(includes o112 p127)(includes o112 p188)(includes o112 p255)(includes o112 p280)

(waiting o113)
(includes o113 p12)(includes o113 p32)(includes o113 p41)(includes o113 p88)(includes o113 p106)(includes o113 p117)(includes o113 p135)(includes o113 p151)(includes o113 p164)(includes o113 p221)(includes o113 p270)

(waiting o114)
(includes o114 p26)(includes o114 p94)(includes o114 p115)(includes o114 p129)(includes o114 p204)

(waiting o115)
(includes o115 p1)(includes o115 p54)(includes o115 p115)(includes o115 p167)(includes o115 p204)(includes o115 p225)(includes o115 p236)(includes o115 p247)(includes o115 p271)

(waiting o116)
(includes o116 p2)(includes o116 p44)(includes o116 p56)(includes o116 p81)(includes o116 p101)(includes o116 p167)(includes o116 p221)(includes o116 p234)(includes o116 p256)(includes o116 p266)

(waiting o117)
(includes o117 p8)(includes o117 p100)(includes o117 p102)(includes o117 p107)(includes o117 p126)(includes o117 p165)(includes o117 p188)(includes o117 p248)

(waiting o118)
(includes o118 p81)(includes o118 p91)(includes o118 p135)

(waiting o119)
(includes o119 p9)(includes o119 p16)(includes o119 p20)(includes o119 p21)(includes o119 p181)(includes o119 p190)(includes o119 p199)(includes o119 p256)

(waiting o120)
(includes o120 p2)(includes o120 p12)(includes o120 p16)(includes o120 p44)(includes o120 p69)(includes o120 p140)(includes o120 p155)(includes o120 p177)(includes o120 p257)

(waiting o121)
(includes o121 p10)(includes o121 p35)(includes o121 p160)(includes o121 p175)(includes o121 p217)(includes o121 p240)

(waiting o122)
(includes o122 p15)(includes o122 p119)(includes o122 p147)(includes o122 p215)(includes o122 p228)

(waiting o123)
(includes o123 p4)(includes o123 p8)(includes o123 p22)(includes o123 p54)(includes o123 p107)(includes o123 p190)(includes o123 p202)(includes o123 p254)(includes o123 p287)(includes o123 p289)(includes o123 p290)

(waiting o124)
(includes o124 p6)(includes o124 p18)(includes o124 p48)(includes o124 p131)(includes o124 p240)(includes o124 p253)

(waiting o125)
(includes o125 p188)(includes o125 p242)(includes o125 p269)

(waiting o126)
(includes o126 p41)(includes o126 p86)(includes o126 p148)(includes o126 p157)(includes o126 p214)(includes o126 p266)

(waiting o127)
(includes o127 p4)(includes o127 p23)(includes o127 p60)(includes o127 p104)(includes o127 p140)(includes o127 p177)(includes o127 p180)(includes o127 p219)

(waiting o128)
(includes o128 p7)(includes o128 p68)(includes o128 p83)(includes o128 p91)(includes o128 p105)(includes o128 p140)(includes o128 p149)(includes o128 p169)(includes o128 p232)(includes o128 p237)(includes o128 p256)(includes o128 p258)

(waiting o129)
(includes o129 p41)(includes o129 p74)(includes o129 p185)(includes o129 p249)(includes o129 p255)(includes o129 p290)

(waiting o130)
(includes o130 p1)(includes o130 p9)(includes o130 p34)(includes o130 p74)(includes o130 p77)(includes o130 p106)(includes o130 p109)(includes o130 p156)(includes o130 p203)(includes o130 p209)(includes o130 p216)(includes o130 p265)

(waiting o131)
(includes o131 p6)(includes o131 p38)(includes o131 p86)(includes o131 p125)(includes o131 p176)(includes o131 p220)(includes o131 p226)(includes o131 p239)(includes o131 p253)(includes o131 p256)(includes o131 p262)(includes o131 p284)(includes o131 p290)

(waiting o132)
(includes o132 p14)(includes o132 p15)(includes o132 p46)(includes o132 p50)(includes o132 p60)(includes o132 p84)(includes o132 p109)(includes o132 p113)(includes o132 p181)(includes o132 p226)(includes o132 p270)

(waiting o133)
(includes o133 p7)(includes o133 p40)(includes o133 p153)(includes o133 p173)(includes o133 p184)

(waiting o134)
(includes o134 p14)(includes o134 p81)(includes o134 p113)(includes o134 p129)(includes o134 p185)(includes o134 p191)(includes o134 p193)(includes o134 p251)

(waiting o135)
(includes o135 p35)(includes o135 p46)(includes o135 p77)(includes o135 p118)(includes o135 p125)(includes o135 p175)(includes o135 p202)(includes o135 p268)

(waiting o136)
(includes o136 p77)(includes o136 p134)(includes o136 p137)(includes o136 p175)(includes o136 p239)

(waiting o137)
(includes o137 p24)(includes o137 p84)(includes o137 p201)

(waiting o138)
(includes o138 p32)(includes o138 p38)(includes o138 p48)(includes o138 p77)(includes o138 p94)(includes o138 p106)(includes o138 p224)(includes o138 p228)(includes o138 p290)

(waiting o139)
(includes o139 p72)(includes o139 p93)(includes o139 p108)(includes o139 p141)(includes o139 p207)

(waiting o140)
(includes o140 p4)(includes o140 p48)(includes o140 p78)(includes o140 p88)(includes o140 p118)(includes o140 p120)(includes o140 p122)(includes o140 p192)(includes o140 p236)

(waiting o141)
(includes o141 p21)(includes o141 p36)(includes o141 p84)(includes o141 p185)(includes o141 p214)(includes o141 p233)(includes o141 p269)

(waiting o142)
(includes o142 p17)(includes o142 p55)(includes o142 p62)(includes o142 p76)(includes o142 p114)(includes o142 p226)

(waiting o143)
(includes o143 p74)(includes o143 p139)(includes o143 p142)(includes o143 p147)(includes o143 p213)(includes o143 p228)(includes o143 p244)(includes o143 p248)

(waiting o144)
(includes o144 p68)(includes o144 p165)(includes o144 p204)(includes o144 p214)(includes o144 p216)(includes o144 p220)(includes o144 p225)(includes o144 p235)(includes o144 p256)

(waiting o145)
(includes o145 p36)(includes o145 p106)(includes o145 p115)(includes o145 p130)(includes o145 p197)

(waiting o146)
(includes o146 p75)(includes o146 p96)(includes o146 p151)(includes o146 p183)(includes o146 p217)(includes o146 p262)

(waiting o147)
(includes o147 p42)(includes o147 p74)(includes o147 p165)(includes o147 p175)(includes o147 p190)(includes o147 p218)(includes o147 p236)(includes o147 p253)(includes o147 p259)

(waiting o148)
(includes o148 p32)(includes o148 p76)(includes o148 p88)(includes o148 p95)(includes o148 p117)(includes o148 p201)(includes o148 p232)(includes o148 p236)(includes o148 p289)

(waiting o149)
(includes o149 p53)(includes o149 p179)(includes o149 p246)(includes o149 p256)

(waiting o150)
(includes o150 p7)(includes o150 p39)(includes o150 p122)(includes o150 p146)(includes o150 p161)(includes o150 p227)

(waiting o151)
(includes o151 p113)(includes o151 p259)(includes o151 p288)

(waiting o152)
(includes o152 p3)(includes o152 p119)(includes o152 p135)(includes o152 p168)(includes o152 p174)(includes o152 p178)(includes o152 p227)(includes o152 p250)

(waiting o153)
(includes o153 p13)(includes o153 p31)(includes o153 p105)(includes o153 p166)(includes o153 p255)(includes o153 p267)

(waiting o154)
(includes o154 p68)(includes o154 p73)(includes o154 p127)(includes o154 p128)(includes o154 p175)(includes o154 p181)(includes o154 p193)(includes o154 p207)(includes o154 p239)(includes o154 p278)

(waiting o155)
(includes o155 p8)(includes o155 p71)(includes o155 p104)(includes o155 p137)(includes o155 p244)(includes o155 p261)(includes o155 p282)

(waiting o156)
(includes o156 p24)(includes o156 p75)(includes o156 p179)(includes o156 p199)(includes o156 p202)(includes o156 p211)(includes o156 p262)

(waiting o157)
(includes o157 p8)(includes o157 p36)(includes o157 p83)(includes o157 p118)(includes o157 p147)(includes o157 p173)(includes o157 p268)(includes o157 p274)

(waiting o158)
(includes o158 p26)(includes o158 p35)(includes o158 p40)(includes o158 p156)(includes o158 p184)(includes o158 p197)(includes o158 p239)

(waiting o159)
(includes o159 p128)(includes o159 p187)(includes o159 p233)

(waiting o160)
(includes o160 p22)(includes o160 p26)(includes o160 p52)(includes o160 p53)(includes o160 p62)(includes o160 p156)(includes o160 p187)(includes o160 p235)

(waiting o161)
(includes o161 p27)(includes o161 p117)(includes o161 p126)(includes o161 p135)(includes o161 p168)(includes o161 p233)

(waiting o162)
(includes o162 p11)(includes o162 p18)(includes o162 p80)(includes o162 p84)(includes o162 p113)(includes o162 p150)(includes o162 p187)(includes o162 p249)

(waiting o163)
(includes o163 p23)(includes o163 p98)(includes o163 p126)(includes o163 p139)(includes o163 p190)(includes o163 p212)

(waiting o164)
(includes o164 p17)(includes o164 p49)(includes o164 p135)(includes o164 p139)(includes o164 p186)(includes o164 p217)(includes o164 p232)(includes o164 p256)(includes o164 p273)

(waiting o165)
(includes o165 p35)(includes o165 p187)(includes o165 p265)

(waiting o166)
(includes o166 p7)(includes o166 p45)(includes o166 p112)(includes o166 p123)(includes o166 p206)(includes o166 p247)

(waiting o167)
(includes o167 p16)(includes o167 p55)(includes o167 p78)(includes o167 p156)(includes o167 p191)

(waiting o168)
(includes o168 p25)(includes o168 p100)(includes o168 p172)(includes o168 p192)(includes o168 p223)(includes o168 p249)(includes o168 p271)(includes o168 p279)(includes o168 p289)

(waiting o169)
(includes o169 p2)(includes o169 p9)(includes o169 p109)(includes o169 p137)(includes o169 p142)(includes o169 p162)(includes o169 p193)(includes o169 p200)(includes o169 p212)

(waiting o170)
(includes o170 p118)(includes o170 p172)(includes o170 p256)

(waiting o171)
(includes o171 p18)(includes o171 p31)(includes o171 p37)(includes o171 p101)(includes o171 p115)(includes o171 p167)(includes o171 p194)(includes o171 p218)

(waiting o172)
(includes o172 p24)(includes o172 p128)(includes o172 p211)(includes o172 p246)(includes o172 p261)

(waiting o173)
(includes o173 p68)(includes o173 p73)(includes o173 p83)(includes o173 p114)(includes o173 p165)(includes o173 p203)(includes o173 p240)

(waiting o174)
(includes o174 p89)(includes o174 p126)(includes o174 p144)(includes o174 p244)(includes o174 p252)(includes o174 p257)(includes o174 p280)

(waiting o175)
(includes o175 p36)(includes o175 p47)(includes o175 p83)(includes o175 p89)(includes o175 p111)(includes o175 p154)(includes o175 p224)(includes o175 p231)(includes o175 p255)

(waiting o176)
(includes o176 p167)(includes o176 p226)(includes o176 p244)(includes o176 p246)

(waiting o177)
(includes o177 p53)(includes o177 p67)(includes o177 p90)(includes o177 p204)(includes o177 p244)(includes o177 p280)

(waiting o178)
(includes o178 p60)(includes o178 p102)(includes o178 p169)(includes o178 p197)(includes o178 p236)

(waiting o179)
(includes o179 p9)(includes o179 p130)(includes o179 p139)(includes o179 p151)(includes o179 p209)(includes o179 p281)(includes o179 p288)

(waiting o180)
(includes o180 p1)(includes o180 p12)(includes o180 p22)(includes o180 p66)(includes o180 p82)(includes o180 p88)(includes o180 p168)(includes o180 p193)(includes o180 p235)

(waiting o181)
(includes o181 p2)(includes o181 p20)(includes o181 p65)(includes o181 p103)(includes o181 p108)(includes o181 p213)(includes o181 p252)(includes o181 p262)(includes o181 p275)

(waiting o182)
(includes o182 p65)(includes o182 p96)(includes o182 p100)(includes o182 p139)(includes o182 p165)(includes o182 p223)(includes o182 p231)(includes o182 p263)(includes o182 p278)

(waiting o183)
(includes o183 p3)(includes o183 p99)(includes o183 p120)(includes o183 p182)(includes o183 p284)

(waiting o184)
(includes o184 p14)(includes o184 p22)(includes o184 p28)(includes o184 p44)(includes o184 p118)(includes o184 p146)(includes o184 p189)(includes o184 p200)(includes o184 p202)

(waiting o185)
(includes o185 p9)(includes o185 p79)(includes o185 p180)(includes o185 p237)(includes o185 p257)

(waiting o186)
(includes o186 p71)(includes o186 p74)(includes o186 p84)(includes o186 p156)(includes o186 p197)(includes o186 p214)(includes o186 p240)(includes o186 p252)

(waiting o187)
(includes o187 p28)(includes o187 p47)(includes o187 p113)(includes o187 p122)(includes o187 p126)(includes o187 p169)(includes o187 p182)(includes o187 p235)(includes o187 p239)(includes o187 p246)(includes o187 p249)

(waiting o188)
(includes o188 p6)(includes o188 p58)(includes o188 p247)

(waiting o189)
(includes o189 p77)(includes o189 p90)(includes o189 p105)(includes o189 p118)(includes o189 p122)(includes o189 p159)(includes o189 p170)

(waiting o190)
(includes o190 p21)(includes o190 p69)(includes o190 p88)(includes o190 p111)(includes o190 p114)(includes o190 p164)(includes o190 p193)(includes o190 p239)(includes o190 p264)

(waiting o191)
(includes o191 p15)(includes o191 p46)(includes o191 p75)(includes o191 p80)(includes o191 p134)(includes o191 p137)(includes o191 p192)(includes o191 p209)(includes o191 p266)

(waiting o192)
(includes o192 p14)(includes o192 p33)(includes o192 p60)(includes o192 p118)(includes o192 p137)(includes o192 p142)(includes o192 p168)(includes o192 p188)(includes o192 p219)(includes o192 p234)

(waiting o193)
(includes o193 p82)(includes o193 p107)(includes o193 p115)(includes o193 p209)(includes o193 p216)(includes o193 p250)(includes o193 p283)

(waiting o194)
(includes o194 p123)(includes o194 p156)(includes o194 p164)(includes o194 p176)(includes o194 p237)(includes o194 p281)

(waiting o195)
(includes o195 p4)(includes o195 p16)(includes o195 p59)(includes o195 p74)(includes o195 p75)(includes o195 p179)(includes o195 p209)(includes o195 p231)(includes o195 p252)(includes o195 p253)(includes o195 p283)

(waiting o196)
(includes o196 p16)(includes o196 p50)(includes o196 p70)(includes o196 p93)(includes o196 p101)(includes o196 p118)(includes o196 p119)(includes o196 p140)(includes o196 p167)(includes o196 p246)(includes o196 p267)(includes o196 p276)

(waiting o197)
(includes o197 p156)(includes o197 p172)(includes o197 p185)(includes o197 p211)(includes o197 p233)(includes o197 p240)(includes o197 p243)(includes o197 p290)

(waiting o198)
(includes o198 p39)(includes o198 p46)(includes o198 p52)(includes o198 p84)(includes o198 p109)(includes o198 p151)(includes o198 p170)(includes o198 p181)(includes o198 p253)(includes o198 p275)

(waiting o199)
(includes o199 p26)(includes o199 p32)(includes o199 p96)(includes o199 p125)(includes o199 p208)(includes o199 p228)(includes o199 p262)(includes o199 p269)(includes o199 p272)(includes o199 p284)

(waiting o200)
(includes o200 p50)(includes o200 p169)(includes o200 p283)

(waiting o201)
(includes o201 p43)(includes o201 p87)(includes o201 p122)(includes o201 p159)(includes o201 p163)(includes o201 p171)(includes o201 p205)(includes o201 p262)(includes o201 p264)

(waiting o202)
(includes o202 p20)(includes o202 p32)(includes o202 p34)(includes o202 p141)(includes o202 p189)(includes o202 p203)(includes o202 p272)

(waiting o203)
(includes o203 p48)(includes o203 p102)(includes o203 p108)(includes o203 p140)(includes o203 p218)(includes o203 p238)(includes o203 p256)(includes o203 p257)

(waiting o204)
(includes o204 p88)(includes o204 p105)(includes o204 p112)(includes o204 p133)(includes o204 p213)(includes o204 p283)

(waiting o205)
(includes o205 p116)(includes o205 p163)(includes o205 p194)(includes o205 p198)(includes o205 p214)(includes o205 p225)

(waiting o206)
(includes o206 p82)(includes o206 p115)(includes o206 p176)

(waiting o207)
(includes o207 p61)(includes o207 p74)(includes o207 p159)(includes o207 p188)(includes o207 p194)(includes o207 p241)(includes o207 p243)(includes o207 p267)(includes o207 p286)

(waiting o208)
(includes o208 p58)(includes o208 p122)(includes o208 p150)(includes o208 p273)(includes o208 p276)

(waiting o209)
(includes o209 p12)(includes o209 p103)(includes o209 p120)(includes o209 p184)(includes o209 p199)(includes o209 p201)(includes o209 p267)(includes o209 p277)

(waiting o210)
(includes o210 p16)(includes o210 p22)(includes o210 p41)(includes o210 p52)(includes o210 p148)(includes o210 p169)(includes o210 p194)(includes o210 p196)(includes o210 p251)

(waiting o211)
(includes o211 p5)(includes o211 p6)(includes o211 p34)(includes o211 p79)(includes o211 p89)(includes o211 p116)(includes o211 p122)(includes o211 p124)(includes o211 p186)(includes o211 p201)(includes o211 p251)

(waiting o212)
(includes o212 p89)(includes o212 p110)(includes o212 p146)(includes o212 p219)(includes o212 p239)(includes o212 p240)

(waiting o213)
(includes o213 p37)(includes o213 p66)(includes o213 p148)(includes o213 p162)(includes o213 p256)

(waiting o214)
(includes o214 p38)(includes o214 p55)(includes o214 p58)(includes o214 p66)(includes o214 p100)(includes o214 p113)(includes o214 p145)(includes o214 p148)(includes o214 p156)(includes o214 p241)(includes o214 p270)(includes o214 p280)

(waiting o215)
(includes o215 p34)(includes o215 p65)(includes o215 p87)(includes o215 p176)(includes o215 p238)(includes o215 p243)(includes o215 p288)(includes o215 p289)

(waiting o216)
(includes o216 p6)(includes o216 p28)(includes o216 p56)(includes o216 p73)(includes o216 p91)(includes o216 p116)(includes o216 p121)(includes o216 p138)(includes o216 p149)(includes o216 p155)(includes o216 p161)(includes o216 p196)(includes o216 p244)

(waiting o217)
(includes o217 p15)(includes o217 p45)(includes o217 p54)(includes o217 p57)(includes o217 p205)

(waiting o218)
(includes o218 p16)(includes o218 p19)(includes o218 p31)(includes o218 p60)(includes o218 p98)(includes o218 p155)(includes o218 p253)(includes o218 p269)(includes o218 p271)(includes o218 p289)

(waiting o219)
(includes o219 p48)(includes o219 p94)(includes o219 p119)(includes o219 p133)(includes o219 p153)(includes o219 p171)(includes o219 p206)(includes o219 p230)(includes o219 p282)

(waiting o220)
(includes o220 p188)

(waiting o221)
(includes o221 p23)(includes o221 p31)(includes o221 p99)(includes o221 p195)(includes o221 p203)(includes o221 p206)(includes o221 p242)(includes o221 p274)(includes o221 p275)

(waiting o222)
(includes o222 p49)(includes o222 p79)(includes o222 p103)(includes o222 p195)(includes o222 p236)(includes o222 p257)(includes o222 p270)

(waiting o223)
(includes o223 p31)(includes o223 p90)(includes o223 p107)(includes o223 p153)(includes o223 p268)

(waiting o224)
(includes o224 p30)(includes o224 p55)(includes o224 p58)(includes o224 p102)(includes o224 p163)(includes o224 p173)(includes o224 p212)(includes o224 p287)

(waiting o225)
(includes o225 p4)(includes o225 p20)(includes o225 p62)(includes o225 p84)(includes o225 p144)(includes o225 p164)(includes o225 p197)(includes o225 p209)(includes o225 p225)

(waiting o226)
(includes o226 p62)(includes o226 p97)(includes o226 p152)(includes o226 p211)(includes o226 p220)(includes o226 p224)(includes o226 p227)(includes o226 p250)(includes o226 p281)

(waiting o227)
(includes o227 p53)(includes o227 p69)(includes o227 p103)(includes o227 p174)(includes o227 p178)(includes o227 p240)(includes o227 p241)(includes o227 p251)

(waiting o228)
(includes o228 p58)(includes o228 p146)(includes o228 p241)(includes o228 p270)

(waiting o229)
(includes o229 p55)(includes o229 p62)(includes o229 p116)(includes o229 p134)(includes o229 p143)(includes o229 p166)(includes o229 p193)

(waiting o230)
(includes o230 p110)(includes o230 p194)(includes o230 p208)(includes o230 p216)

(waiting o231)
(includes o231 p24)(includes o231 p75)(includes o231 p95)(includes o231 p164)(includes o231 p211)(includes o231 p261)(includes o231 p289)

(waiting o232)
(includes o232 p39)(includes o232 p64)(includes o232 p79)(includes o232 p164)(includes o232 p217)(includes o232 p224)(includes o232 p228)(includes o232 p247)(includes o232 p278)

(waiting o233)
(includes o233 p16)(includes o233 p146)(includes o233 p169)(includes o233 p241)(includes o233 p246)(includes o233 p252)

(waiting o234)
(includes o234 p90)(includes o234 p142)(includes o234 p234)(includes o234 p254)(includes o234 p268)(includes o234 p273)(includes o234 p284)(includes o234 p290)

(waiting o235)
(includes o235 p31)(includes o235 p75)(includes o235 p228)(includes o235 p232)(includes o235 p264)(includes o235 p269)(includes o235 p272)(includes o235 p287)

(waiting o236)
(includes o236 p46)(includes o236 p51)(includes o236 p101)(includes o236 p113)(includes o236 p196)(includes o236 p212)(includes o236 p230)

(waiting o237)
(includes o237 p109)(includes o237 p124)(includes o237 p202)(includes o237 p213)

(waiting o238)
(includes o238 p107)(includes o238 p133)(includes o238 p171)(includes o238 p228)(includes o238 p248)(includes o238 p263)(includes o238 p280)

(waiting o239)
(includes o239 p3)(includes o239 p20)(includes o239 p78)(includes o239 p122)(includes o239 p133)(includes o239 p159)

(waiting o240)
(includes o240 p37)(includes o240 p59)(includes o240 p105)(includes o240 p151)(includes o240 p156)(includes o240 p161)

(waiting o241)
(includes o241 p32)(includes o241 p73)(includes o241 p76)(includes o241 p91)(includes o241 p121)(includes o241 p135)(includes o241 p156)(includes o241 p164)(includes o241 p232)

(waiting o242)
(includes o242 p4)(includes o242 p49)(includes o242 p64)(includes o242 p84)(includes o242 p112)(includes o242 p125)(includes o242 p215)(includes o242 p225)(includes o242 p252)(includes o242 p277)(includes o242 p284)(includes o242 p285)

(waiting o243)
(includes o243 p106)(includes o243 p123)(includes o243 p124)(includes o243 p128)(includes o243 p132)(includes o243 p138)(includes o243 p192)(includes o243 p210)(includes o243 p218)(includes o243 p240)(includes o243 p280)

(waiting o244)
(includes o244 p36)(includes o244 p54)(includes o244 p90)

(waiting o245)
(includes o245 p59)(includes o245 p83)(includes o245 p136)(includes o245 p138)(includes o245 p189)(includes o245 p204)(includes o245 p211)(includes o245 p233)(includes o245 p266)(includes o245 p269)

(waiting o246)
(includes o246 p13)(includes o246 p34)(includes o246 p62)(includes o246 p135)(includes o246 p170)(includes o246 p193)(includes o246 p196)(includes o246 p197)(includes o246 p201)(includes o246 p277)

(waiting o247)
(includes o247 p14)(includes o247 p82)(includes o247 p88)(includes o247 p101)(includes o247 p108)(includes o247 p173)(includes o247 p201)(includes o247 p239)(includes o247 p290)

(waiting o248)
(includes o248 p16)(includes o248 p32)(includes o248 p57)(includes o248 p160)(includes o248 p168)(includes o248 p201)(includes o248 p216)

(waiting o249)
(includes o249 p4)(includes o249 p62)(includes o249 p86)(includes o249 p87)(includes o249 p103)

(waiting o250)
(includes o250 p24)(includes o250 p28)(includes o250 p39)(includes o250 p95)(includes o250 p99)(includes o250 p127)(includes o250 p182)(includes o250 p197)(includes o250 p261)

(waiting o251)
(includes o251 p14)(includes o251 p17)(includes o251 p175)

(waiting o252)
(includes o252 p9)(includes o252 p10)(includes o252 p130)(includes o252 p173)(includes o252 p201)(includes o252 p244)(includes o252 p270)(includes o252 p282)(includes o252 p285)

(waiting o253)
(includes o253 p9)(includes o253 p23)(includes o253 p40)(includes o253 p100)(includes o253 p113)(includes o253 p146)(includes o253 p210)(includes o253 p263)

(waiting o254)
(includes o254 p16)(includes o254 p37)(includes o254 p48)(includes o254 p98)(includes o254 p154)(includes o254 p163)(includes o254 p170)(includes o254 p208)(includes o254 p257)

(waiting o255)
(includes o255 p46)(includes o255 p146)(includes o255 p229)(includes o255 p252)(includes o255 p273)(includes o255 p282)

(waiting o256)
(includes o256 p17)(includes o256 p30)(includes o256 p40)(includes o256 p50)(includes o256 p116)(includes o256 p123)(includes o256 p133)(includes o256 p230)(includes o256 p270)(includes o256 p278)

(waiting o257)
(includes o257 p21)(includes o257 p26)(includes o257 p42)(includes o257 p186)(includes o257 p213)(includes o257 p218)(includes o257 p225)(includes o257 p266)(includes o257 p274)

(waiting o258)
(includes o258 p35)(includes o258 p40)(includes o258 p163)(includes o258 p193)(includes o258 p257)

(waiting o259)
(includes o259 p62)(includes o259 p63)(includes o259 p105)(includes o259 p177)(includes o259 p214)(includes o259 p249)(includes o259 p263)(includes o259 p286)

(waiting o260)
(includes o260 p71)(includes o260 p72)(includes o260 p110)(includes o260 p121)(includes o260 p132)(includes o260 p144)(includes o260 p176)(includes o260 p178)(includes o260 p202)(includes o260 p223)

(waiting o261)
(includes o261 p61)(includes o261 p79)(includes o261 p173)(includes o261 p210)(includes o261 p230)(includes o261 p285)

(waiting o262)
(includes o262 p60)(includes o262 p93)(includes o262 p111)(includes o262 p112)(includes o262 p196)(includes o262 p208)(includes o262 p215)(includes o262 p238)(includes o262 p239)(includes o262 p268)(includes o262 p271)

(waiting o263)
(includes o263 p59)(includes o263 p85)(includes o263 p91)(includes o263 p107)(includes o263 p113)(includes o263 p177)(includes o263 p183)(includes o263 p233)

(waiting o264)
(includes o264 p7)(includes o264 p46)(includes o264 p114)(includes o264 p146)(includes o264 p169)(includes o264 p179)(includes o264 p270)(includes o264 p279)

(waiting o265)
(includes o265 p19)(includes o265 p49)(includes o265 p96)(includes o265 p102)(includes o265 p108)(includes o265 p119)(includes o265 p136)(includes o265 p141)(includes o265 p196)(includes o265 p282)

(waiting o266)
(includes o266 p52)(includes o266 p186)

(waiting o267)
(includes o267 p10)(includes o267 p21)(includes o267 p34)(includes o267 p53)(includes o267 p82)(includes o267 p134)(includes o267 p209)(includes o267 p253)(includes o267 p288)

(waiting o268)
(includes o268 p29)(includes o268 p91)(includes o268 p103)(includes o268 p146)(includes o268 p151)(includes o268 p166)(includes o268 p198)(includes o268 p216)(includes o268 p256)(includes o268 p281)(includes o268 p290)

(waiting o269)
(includes o269 p114)(includes o269 p194)(includes o269 p245)

(waiting o270)
(includes o270 p44)(includes o270 p47)(includes o270 p56)(includes o270 p106)(includes o270 p235)(includes o270 p278)

(waiting o271)
(includes o271 p129)(includes o271 p140)(includes o271 p159)(includes o271 p273)(includes o271 p275)

(waiting o272)
(includes o272 p19)(includes o272 p154)(includes o272 p167)(includes o272 p195)(includes o272 p234)(includes o272 p263)(includes o272 p277)(includes o272 p283)

(waiting o273)
(includes o273 p56)(includes o273 p89)(includes o273 p107)(includes o273 p176)(includes o273 p192)(includes o273 p264)(includes o273 p283)

(waiting o274)
(includes o274 p37)(includes o274 p62)(includes o274 p95)(includes o274 p125)(includes o274 p130)(includes o274 p152)(includes o274 p187)

(waiting o275)
(includes o275 p1)(includes o275 p109)(includes o275 p139)(includes o275 p192)(includes o275 p219)(includes o275 p223)(includes o275 p275)

(waiting o276)
(includes o276 p1)(includes o276 p87)(includes o276 p104)(includes o276 p160)(includes o276 p194)(includes o276 p277)

(waiting o277)
(includes o277 p5)(includes o277 p46)(includes o277 p85)(includes o277 p105)(includes o277 p109)(includes o277 p201)(includes o277 p205)(includes o277 p208)(includes o277 p230)(includes o277 p231)(includes o277 p280)

(waiting o278)
(includes o278 p13)(includes o278 p85)(includes o278 p192)(includes o278 p201)(includes o278 p202)(includes o278 p239)(includes o278 p259)(includes o278 p281)

(waiting o279)
(includes o279 p8)(includes o279 p36)(includes o279 p52)(includes o279 p71)(includes o279 p82)(includes o279 p85)(includes o279 p130)(includes o279 p188)(includes o279 p287)

(waiting o280)
(includes o280 p14)(includes o280 p16)(includes o280 p35)(includes o280 p58)(includes o280 p105)(includes o280 p124)(includes o280 p151)(includes o280 p162)(includes o280 p175)(includes o280 p275)(includes o280 p289)

(waiting o281)
(includes o281 p77)(includes o281 p111)(includes o281 p196)

(waiting o282)
(includes o282 p104)(includes o282 p163)(includes o282 p234)(includes o282 p247)(includes o282 p283)(includes o282 p290)

(waiting o283)
(includes o283 p3)(includes o283 p6)(includes o283 p55)(includes o283 p61)(includes o283 p123)(includes o283 p129)(includes o283 p136)(includes o283 p202)(includes o283 p239)

(waiting o284)
(includes o284 p47)(includes o284 p63)(includes o284 p69)(includes o284 p218)(includes o284 p220)

(waiting o285)
(includes o285 p67)(includes o285 p112)(includes o285 p223)(includes o285 p282)

(waiting o286)
(includes o286 p11)(includes o286 p31)(includes o286 p53)(includes o286 p77)(includes o286 p96)(includes o286 p117)(includes o286 p122)(includes o286 p123)(includes o286 p210)(includes o286 p246)(includes o286 p267)(includes o286 p283)

(waiting o287)
(includes o287 p73)(includes o287 p141)(includes o287 p159)(includes o287 p249)(includes o287 p269)

(waiting o288)
(includes o288 p9)(includes o288 p38)(includes o288 p83)

(waiting o289)
(includes o289 p75)(includes o289 p86)(includes o289 p100)(includes o289 p134)(includes o289 p217)(includes o289 p225)

(waiting o290)
(includes o290 p32)(includes o290 p132)(includes o290 p209)(includes o290 p287)

(not-made p1)
(not-made p2)
(not-made p3)
(not-made p4)
(not-made p5)
(not-made p6)
(not-made p7)
(not-made p8)
(not-made p9)
(not-made p10)
(not-made p11)
(not-made p12)
(not-made p13)
(not-made p14)
(not-made p15)
(not-made p16)
(not-made p17)
(not-made p18)
(not-made p19)
(not-made p20)
(not-made p21)
(not-made p22)
(not-made p23)
(not-made p24)
(not-made p25)
(not-made p26)
(not-made p27)
(not-made p28)
(not-made p29)
(not-made p30)
(not-made p31)
(not-made p32)
(not-made p33)
(not-made p34)
(not-made p35)
(not-made p36)
(not-made p37)
(not-made p38)
(not-made p39)
(not-made p40)
(not-made p41)
(not-made p42)
(not-made p43)
(not-made p44)
(not-made p45)
(not-made p46)
(not-made p47)
(not-made p48)
(not-made p49)
(not-made p50)
(not-made p51)
(not-made p52)
(not-made p53)
(not-made p54)
(not-made p55)
(not-made p56)
(not-made p57)
(not-made p58)
(not-made p59)
(not-made p60)
(not-made p61)
(not-made p62)
(not-made p63)
(not-made p64)
(not-made p65)
(not-made p66)
(not-made p67)
(not-made p68)
(not-made p69)
(not-made p70)
(not-made p71)
(not-made p72)
(not-made p73)
(not-made p74)
(not-made p75)
(not-made p76)
(not-made p77)
(not-made p78)
(not-made p79)
(not-made p80)
(not-made p81)
(not-made p82)
(not-made p83)
(not-made p84)
(not-made p85)
(not-made p86)
(not-made p87)
(not-made p88)
(not-made p89)
(not-made p90)
(not-made p91)
(not-made p92)
(not-made p93)
(not-made p94)
(not-made p95)
(not-made p96)
(not-made p97)
(not-made p98)
(not-made p99)
(not-made p100)
(not-made p101)
(not-made p102)
(not-made p103)
(not-made p104)
(not-made p105)
(not-made p106)
(not-made p107)
(not-made p108)
(not-made p109)
(not-made p110)
(not-made p111)
(not-made p112)
(not-made p113)
(not-made p114)
(not-made p115)
(not-made p116)
(not-made p117)
(not-made p118)
(not-made p119)
(not-made p120)
(not-made p121)
(not-made p122)
(not-made p123)
(not-made p124)
(not-made p125)
(not-made p126)
(not-made p127)
(not-made p128)
(not-made p129)
(not-made p130)
(not-made p131)
(not-made p132)
(not-made p133)
(not-made p134)
(not-made p135)
(not-made p136)
(not-made p137)
(not-made p138)
(not-made p139)
(not-made p140)
(not-made p141)
(not-made p142)
(not-made p143)
(not-made p144)
(not-made p145)
(not-made p146)
(not-made p147)
(not-made p148)
(not-made p149)
(not-made p150)
(not-made p151)
(not-made p152)
(not-made p153)
(not-made p154)
(not-made p155)
(not-made p156)
(not-made p157)
(not-made p158)
(not-made p159)
(not-made p160)
(not-made p161)
(not-made p162)
(not-made p163)
(not-made p164)
(not-made p165)
(not-made p166)
(not-made p167)
(not-made p168)
(not-made p169)
(not-made p170)
(not-made p171)
(not-made p172)
(not-made p173)
(not-made p174)
(not-made p175)
(not-made p176)
(not-made p177)
(not-made p178)
(not-made p179)
(not-made p180)
(not-made p181)
(not-made p182)
(not-made p183)
(not-made p184)
(not-made p185)
(not-made p186)
(not-made p187)
(not-made p188)
(not-made p189)
(not-made p190)
(not-made p191)
(not-made p192)
(not-made p193)
(not-made p194)
(not-made p195)
(not-made p196)
(not-made p197)
(not-made p198)
(not-made p199)
(not-made p200)
(not-made p201)
(not-made p202)
(not-made p203)
(not-made p204)
(not-made p205)
(not-made p206)
(not-made p207)
(not-made p208)
(not-made p209)
(not-made p210)
(not-made p211)
(not-made p212)
(not-made p213)
(not-made p214)
(not-made p215)
(not-made p216)
(not-made p217)
(not-made p218)
(not-made p219)
(not-made p220)
(not-made p221)
(not-made p222)
(not-made p223)
(not-made p224)
(not-made p225)
(not-made p226)
(not-made p227)
(not-made p228)
(not-made p229)
(not-made p230)
(not-made p231)
(not-made p232)
(not-made p233)
(not-made p234)
(not-made p235)
(not-made p236)
(not-made p237)
(not-made p238)
(not-made p239)
(not-made p240)
(not-made p241)
(not-made p242)
(not-made p243)
(not-made p244)
(not-made p245)
(not-made p246)
(not-made p247)
(not-made p248)
(not-made p249)
(not-made p250)
(not-made p251)
(not-made p252)
(not-made p253)
(not-made p254)
(not-made p255)
(not-made p256)
(not-made p257)
(not-made p258)
(not-made p259)
(not-made p260)
(not-made p261)
(not-made p262)
(not-made p263)
(not-made p264)
(not-made p265)
(not-made p266)
(not-made p267)
(not-made p268)
(not-made p269)
(not-made p270)
(not-made p271)
(not-made p272)
(not-made p273)
(not-made p274)
(not-made p275)
(not-made p276)
(not-made p277)
(not-made p278)
(not-made p279)
(not-made p280)
(not-made p281)
(not-made p282)
(not-made p283)
(not-made p284)
(not-made p285)
(not-made p286)
(not-made p287)
(not-made p288)
(not-made p289)
(not-made p290)

(= (total-cost) 0)

)

(:goal
(and
(shipped o1)
(shipped o2)
(shipped o3)
(shipped o4)
(shipped o5)
(shipped o6)
(shipped o7)
(shipped o8)
(shipped o9)
(shipped o10)
(shipped o11)
(shipped o12)
(shipped o13)
(shipped o14)
(shipped o15)
(shipped o16)
(shipped o17)
(shipped o18)
(shipped o19)
(shipped o20)
(shipped o21)
(shipped o22)
(shipped o23)
(shipped o24)
(shipped o25)
(shipped o26)
(shipped o27)
(shipped o28)
(shipped o29)
(shipped o30)
(shipped o31)
(shipped o32)
(shipped o33)
(shipped o34)
(shipped o35)
(shipped o36)
(shipped o37)
(shipped o38)
(shipped o39)
(shipped o40)
(shipped o41)
(shipped o42)
(shipped o43)
(shipped o44)
(shipped o45)
(shipped o46)
(shipped o47)
(shipped o48)
(shipped o49)
(shipped o50)
(shipped o51)
(shipped o52)
(shipped o53)
(shipped o54)
(shipped o55)
(shipped o56)
(shipped o57)
(shipped o58)
(shipped o59)
(shipped o60)
(shipped o61)
(shipped o62)
(shipped o63)
(shipped o64)
(shipped o65)
(shipped o66)
(shipped o67)
(shipped o68)
(shipped o69)
(shipped o70)
(shipped o71)
(shipped o72)
(shipped o73)
(shipped o74)
(shipped o75)
(shipped o76)
(shipped o77)
(shipped o78)
(shipped o79)
(shipped o80)
(shipped o81)
(shipped o82)
(shipped o83)
(shipped o84)
(shipped o85)
(shipped o86)
(shipped o87)
(shipped o88)
(shipped o89)
(shipped o90)
(shipped o91)
(shipped o92)
(shipped o93)
(shipped o94)
(shipped o95)
(shipped o96)
(shipped o97)
(shipped o98)
(shipped o99)
(shipped o100)
(shipped o101)
(shipped o102)
(shipped o103)
(shipped o104)
(shipped o105)
(shipped o106)
(shipped o107)
(shipped o108)
(shipped o109)
(shipped o110)
(shipped o111)
(shipped o112)
(shipped o113)
(shipped o114)
(shipped o115)
(shipped o116)
(shipped o117)
(shipped o118)
(shipped o119)
(shipped o120)
(shipped o121)
(shipped o122)
(shipped o123)
(shipped o124)
(shipped o125)
(shipped o126)
(shipped o127)
(shipped o128)
(shipped o129)
(shipped o130)
(shipped o131)
(shipped o132)
(shipped o133)
(shipped o134)
(shipped o135)
(shipped o136)
(shipped o137)
(shipped o138)
(shipped o139)
(shipped o140)
(shipped o141)
(shipped o142)
(shipped o143)
(shipped o144)
(shipped o145)
(shipped o146)
(shipped o147)
(shipped o148)
(shipped o149)
(shipped o150)
(shipped o151)
(shipped o152)
(shipped o153)
(shipped o154)
(shipped o155)
(shipped o156)
(shipped o157)
(shipped o158)
(shipped o159)
(shipped o160)
(shipped o161)
(shipped o162)
(shipped o163)
(shipped o164)
(shipped o165)
(shipped o166)
(shipped o167)
(shipped o168)
(shipped o169)
(shipped o170)
(shipped o171)
(shipped o172)
(shipped o173)
(shipped o174)
(shipped o175)
(shipped o176)
(shipped o177)
(shipped o178)
(shipped o179)
(shipped o180)
(shipped o181)
(shipped o182)
(shipped o183)
(shipped o184)
(shipped o185)
(shipped o186)
(shipped o187)
(shipped o188)
(shipped o189)
(shipped o190)
(shipped o191)
(shipped o192)
(shipped o193)
(shipped o194)
(shipped o195)
(shipped o196)
(shipped o197)
(shipped o198)
(shipped o199)
(shipped o200)
(shipped o201)
(shipped o202)
(shipped o203)
(shipped o204)
(shipped o205)
(shipped o206)
(shipped o207)
(shipped o208)
(shipped o209)
(shipped o210)
(shipped o211)
(shipped o212)
(shipped o213)
(shipped o214)
(shipped o215)
(shipped o216)
(shipped o217)
(shipped o218)
(shipped o219)
(shipped o220)
(shipped o221)
(shipped o222)
(shipped o223)
(shipped o224)
(shipped o225)
(shipped o226)
(shipped o227)
(shipped o228)
(shipped o229)
(shipped o230)
(shipped o231)
(shipped o232)
(shipped o233)
(shipped o234)
(shipped o235)
(shipped o236)
(shipped o237)
(shipped o238)
(shipped o239)
(shipped o240)
(shipped o241)
(shipped o242)
(shipped o243)
(shipped o244)
(shipped o245)
(shipped o246)
(shipped o247)
(shipped o248)
(shipped o249)
(shipped o250)
(shipped o251)
(shipped o252)
(shipped o253)
(shipped o254)
(shipped o255)
(shipped o256)
(shipped o257)
(shipped o258)
(shipped o259)
(shipped o260)
(shipped o261)
(shipped o262)
(shipped o263)
(shipped o264)
(shipped o265)
(shipped o266)
(shipped o267)
(shipped o268)
(shipped o269)
(shipped o270)
(shipped o271)
(shipped o272)
(shipped o273)
(shipped o274)
(shipped o275)
(shipped o276)
(shipped o277)
(shipped o278)
(shipped o279)
(shipped o280)
(shipped o281)
(shipped o282)
(shipped o283)
(shipped o284)
(shipped o285)
(shipped o286)
(shipped o287)
(shipped o288)
(shipped o289)
(shipped o290)
))

(:metric minimize (total-cost))

)

