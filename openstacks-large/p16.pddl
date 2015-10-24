(define (problem os-sequencedstrips-p270_1)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210 n211 n212 n213 n214 n215 n216 n217 n218 n219 n220 n221 n222 n223 n224 n225 n226 n227 n228 n229 n230 n231 n232 n233 n234 n235 n236 n237 n238 n239 n240 n241 n242 n243 n244 n245 n246 n247 n248 n249 n250 n251 n252 n253 n254 n255 n256 n257 n258 n259 n260 n261 n262 n263 n264 n265 n266 n267 n268 n269 n270  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) (next-count n210 n211) (next-count n211 n212) (next-count n212 n213) (next-count n213 n214) (next-count n214 n215) (next-count n215 n216) (next-count n216 n217) (next-count n217 n218) (next-count n218 n219) (next-count n219 n220) (next-count n220 n221) (next-count n221 n222) (next-count n222 n223) (next-count n223 n224) (next-count n224 n225) (next-count n225 n226) (next-count n226 n227) (next-count n227 n228) (next-count n228 n229) (next-count n229 n230) (next-count n230 n231) (next-count n231 n232) (next-count n232 n233) (next-count n233 n234) (next-count n234 n235) (next-count n235 n236) (next-count n236 n237) (next-count n237 n238) (next-count n238 n239) (next-count n239 n240) (next-count n240 n241) (next-count n241 n242) (next-count n242 n243) (next-count n243 n244) (next-count n244 n245) (next-count n245 n246) (next-count n246 n247) (next-count n247 n248) (next-count n248 n249) (next-count n249 n250) (next-count n250 n251) (next-count n251 n252) (next-count n252 n253) (next-count n253 n254) (next-count n254 n255) (next-count n255 n256) (next-count n256 n257) (next-count n257 n258) (next-count n258 n259) (next-count n259 n260) (next-count n260 n261) (next-count n261 n262) (next-count n262 n263) (next-count n263 n264) (next-count n264 n265) (next-count n265 n266) (next-count n266 n267) (next-count n267 n268) (next-count n268 n269) (next-count n269 n270) 
(stacks-avail n0)

(waiting o1)
(includes o1 p4)(includes o1 p13)(includes o1 p40)(includes o1 p58)(includes o1 p72)(includes o1 p87)(includes o1 p229)

(waiting o2)
(includes o2 p140)(includes o2 p244)(includes o2 p260)

(waiting o3)
(includes o3 p22)(includes o3 p32)(includes o3 p50)(includes o3 p84)(includes o3 p106)(includes o3 p124)(includes o3 p162)(includes o3 p163)(includes o3 p186)(includes o3 p195)

(waiting o4)
(includes o4 p95)(includes o4 p118)(includes o4 p187)(includes o4 p247)(includes o4 p252)

(waiting o5)
(includes o5 p32)(includes o5 p70)(includes o5 p209)(includes o5 p251)(includes o5 p260)(includes o5 p264)

(waiting o6)
(includes o6 p1)(includes o6 p3)(includes o6 p63)(includes o6 p88)(includes o6 p110)(includes o6 p164)(includes o6 p198)

(waiting o7)
(includes o7 p111)(includes o7 p143)(includes o7 p146)(includes o7 p205)(includes o7 p206)

(waiting o8)
(includes o8 p10)(includes o8 p28)(includes o8 p43)(includes o8 p80)(includes o8 p98)(includes o8 p133)(includes o8 p145)(includes o8 p166)(includes o8 p172)(includes o8 p200)

(waiting o9)
(includes o9 p35)(includes o9 p43)(includes o9 p123)(includes o9 p149)(includes o9 p156)(includes o9 p196)(includes o9 p203)(includes o9 p252)

(waiting o10)
(includes o10 p7)(includes o10 p60)

(waiting o11)
(includes o11 p40)(includes o11 p44)(includes o11 p77)(includes o11 p184)(includes o11 p188)(includes o11 p208)

(waiting o12)
(includes o12 p27)(includes o12 p101)(includes o12 p120)(includes o12 p177)(includes o12 p233)(includes o12 p259)

(waiting o13)
(includes o13 p124)(includes o13 p159)(includes o13 p177)

(waiting o14)
(includes o14 p26)(includes o14 p35)(includes o14 p68)

(waiting o15)
(includes o15 p19)(includes o15 p42)(includes o15 p51)(includes o15 p78)(includes o15 p105)(includes o15 p110)(includes o15 p157)(includes o15 p171)(includes o15 p209)(includes o15 p227)

(waiting o16)
(includes o16 p87)(includes o16 p130)(includes o16 p191)(includes o16 p196)(includes o16 p246)(includes o16 p253)

(waiting o17)
(includes o17 p18)(includes o17 p28)(includes o17 p146)(includes o17 p217)

(waiting o18)
(includes o18 p15)(includes o18 p124)(includes o18 p168)(includes o18 p254)

(waiting o19)
(includes o19 p34)(includes o19 p66)(includes o19 p171)(includes o19 p220)

(waiting o20)
(includes o20 p69)(includes o20 p70)(includes o20 p213)(includes o20 p233)

(waiting o21)
(includes o21 p33)(includes o21 p167)(includes o21 p190)(includes o21 p248)

(waiting o22)
(includes o22 p29)(includes o22 p31)(includes o22 p145)(includes o22 p267)

(waiting o23)
(includes o23 p29)(includes o23 p64)(includes o23 p84)(includes o23 p110)(includes o23 p132)(includes o23 p150)(includes o23 p248)

(waiting o24)
(includes o24 p20)(includes o24 p52)(includes o24 p84)(includes o24 p111)(includes o24 p133)

(waiting o25)
(includes o25 p53)(includes o25 p83)(includes o25 p107)(includes o25 p174)(includes o25 p198)(includes o25 p224)

(waiting o26)
(includes o26 p8)(includes o26 p44)(includes o26 p85)(includes o26 p166)(includes o26 p177)(includes o26 p239)(includes o26 p252)

(waiting o27)
(includes o27 p52)(includes o27 p60)(includes o27 p63)(includes o27 p104)(includes o27 p121)(includes o27 p136)(includes o27 p178)

(waiting o28)
(includes o28 p61)(includes o28 p98)(includes o28 p108)(includes o28 p142)(includes o28 p157)(includes o28 p166)(includes o28 p202)(includes o28 p209)(includes o28 p220)(includes o28 p229)(includes o28 p239)

(waiting o29)
(includes o29 p111)(includes o29 p172)(includes o29 p268)

(waiting o30)
(includes o30 p39)(includes o30 p40)(includes o30 p136)(includes o30 p184)

(waiting o31)
(includes o31 p12)(includes o31 p163)(includes o31 p170)(includes o31 p212)(includes o31 p239)

(waiting o32)
(includes o32 p261)

(waiting o33)
(includes o33 p129)(includes o33 p143)(includes o33 p145)(includes o33 p221)(includes o33 p265)

(waiting o34)
(includes o34 p11)(includes o34 p52)(includes o34 p178)

(waiting o35)
(includes o35 p79)(includes o35 p102)(includes o35 p108)(includes o35 p112)(includes o35 p115)(includes o35 p116)(includes o35 p202)(includes o35 p210)

(waiting o36)
(includes o36 p8)(includes o36 p61)(includes o36 p92)(includes o36 p132)(includes o36 p162)(includes o36 p204)(includes o36 p209)

(waiting o37)
(includes o37 p3)(includes o37 p5)(includes o37 p7)(includes o37 p27)(includes o37 p122)(includes o37 p141)(includes o37 p173)(includes o37 p175)(includes o37 p184)(includes o37 p240)

(waiting o38)
(includes o38 p20)(includes o38 p25)(includes o38 p89)(includes o38 p131)(includes o38 p133)(includes o38 p175)(includes o38 p194)(includes o38 p228)(includes o38 p237)(includes o38 p262)

(waiting o39)
(includes o39 p48)(includes o39 p129)(includes o39 p184)(includes o39 p212)(includes o39 p258)(includes o39 p270)

(waiting o40)
(includes o40 p1)(includes o40 p2)(includes o40 p5)(includes o40 p40)(includes o40 p45)(includes o40 p65)(includes o40 p85)(includes o40 p144)(includes o40 p187)(includes o40 p206)(includes o40 p207)

(waiting o41)
(includes o41 p57)(includes o41 p80)(includes o41 p144)(includes o41 p200)(includes o41 p252)

(waiting o42)
(includes o42 p33)(includes o42 p36)(includes o42 p45)(includes o42 p95)(includes o42 p121)

(waiting o43)
(includes o43 p74)(includes o43 p102)(includes o43 p188)(includes o43 p191)

(waiting o44)
(includes o44 p2)(includes o44 p76)(includes o44 p90)(includes o44 p202)

(waiting o45)
(includes o45 p39)(includes o45 p45)(includes o45 p82)(includes o45 p110)(includes o45 p129)(includes o45 p159)(includes o45 p167)(includes o45 p194)(includes o45 p208)(includes o45 p212)(includes o45 p219)(includes o45 p227)

(waiting o46)
(includes o46 p26)(includes o46 p29)(includes o46 p51)(includes o46 p59)(includes o46 p63)(includes o46 p90)(includes o46 p149)(includes o46 p169)(includes o46 p195)(includes o46 p245)

(waiting o47)
(includes o47 p18)(includes o47 p104)(includes o47 p155)(includes o47 p160)(includes o47 p163)(includes o47 p194)(includes o47 p211)(includes o47 p270)

(waiting o48)
(includes o48 p67)(includes o48 p108)(includes o48 p172)

(waiting o49)
(includes o49 p10)(includes o49 p19)(includes o49 p46)(includes o49 p64)(includes o49 p77)(includes o49 p86)(includes o49 p144)(includes o49 p245)(includes o49 p265)(includes o49 p266)

(waiting o50)
(includes o50 p70)(includes o50 p94)(includes o50 p118)(includes o50 p124)(includes o50 p134)(includes o50 p204)(includes o50 p216)(includes o50 p226)

(waiting o51)
(includes o51 p63)(includes o51 p86)(includes o51 p87)(includes o51 p143)(includes o51 p146)(includes o51 p179)(includes o51 p205)(includes o51 p211)(includes o51 p222)(includes o51 p244)

(waiting o52)
(includes o52 p103)(includes o52 p124)(includes o52 p140)(includes o52 p214)(includes o52 p241)(includes o52 p262)

(waiting o53)
(includes o53 p78)(includes o53 p112)(includes o53 p145)(includes o53 p204)(includes o53 p259)

(waiting o54)
(includes o54 p15)(includes o54 p19)(includes o54 p56)(includes o54 p61)(includes o54 p97)(includes o54 p163)(includes o54 p167)(includes o54 p226)(includes o54 p230)(includes o54 p248)(includes o54 p252)

(waiting o55)
(includes o55 p1)(includes o55 p45)(includes o55 p107)(includes o55 p137)(includes o55 p198)

(waiting o56)
(includes o56 p11)(includes o56 p22)(includes o56 p35)(includes o56 p56)(includes o56 p81)(includes o56 p99)(includes o56 p103)(includes o56 p108)(includes o56 p186)(includes o56 p191)(includes o56 p253)

(waiting o57)
(includes o57 p13)(includes o57 p86)(includes o57 p101)(includes o57 p109)(includes o57 p134)(includes o57 p150)

(waiting o58)
(includes o58 p29)(includes o58 p45)(includes o58 p56)(includes o58 p149)(includes o58 p172)(includes o58 p184)(includes o58 p261)(includes o58 p264)

(waiting o59)
(includes o59 p148)(includes o59 p157)(includes o59 p270)

(waiting o60)
(includes o60 p34)(includes o60 p70)(includes o60 p92)(includes o60 p238)

(waiting o61)
(includes o61 p78)(includes o61 p94)(includes o61 p106)(includes o61 p148)(includes o61 p169)(includes o61 p174)(includes o61 p222)(includes o61 p239)(includes o61 p270)

(waiting o62)
(includes o62 p47)(includes o62 p94)(includes o62 p105)(includes o62 p107)(includes o62 p161)(includes o62 p175)(includes o62 p194)(includes o62 p205)(includes o62 p238)(includes o62 p261)

(waiting o63)
(includes o63 p44)(includes o63 p101)(includes o63 p123)(includes o63 p131)(includes o63 p213)(includes o63 p231)(includes o63 p233)

(waiting o64)
(includes o64 p25)(includes o64 p52)(includes o64 p62)(includes o64 p141)(includes o64 p183)

(waiting o65)
(includes o65 p32)(includes o65 p51)(includes o65 p52)(includes o65 p71)(includes o65 p84)(includes o65 p88)(includes o65 p136)(includes o65 p146)(includes o65 p157)(includes o65 p171)(includes o65 p251)

(waiting o66)
(includes o66 p3)(includes o66 p117)(includes o66 p137)(includes o66 p187)

(waiting o67)
(includes o67 p91)(includes o67 p101)(includes o67 p119)(includes o67 p123)(includes o67 p135)(includes o67 p171)(includes o67 p180)(includes o67 p233)

(waiting o68)
(includes o68 p43)(includes o68 p57)(includes o68 p60)(includes o68 p88)(includes o68 p133)(includes o68 p143)(includes o68 p182)

(waiting o69)
(includes o69 p21)(includes o69 p43)(includes o69 p47)(includes o69 p76)(includes o69 p157)(includes o69 p165)(includes o69 p200)(includes o69 p237)(includes o69 p243)(includes o69 p252)

(waiting o70)
(includes o70 p26)(includes o70 p132)(includes o70 p135)(includes o70 p180)(includes o70 p190)(includes o70 p249)(includes o70 p259)

(waiting o71)
(includes o71 p47)(includes o71 p87)(includes o71 p129)(includes o71 p134)(includes o71 p202)(includes o71 p203)(includes o71 p239)

(waiting o72)
(includes o72 p53)(includes o72 p221)(includes o72 p222)(includes o72 p233)(includes o72 p241)

(waiting o73)
(includes o73 p31)(includes o73 p80)(includes o73 p114)(includes o73 p137)(includes o73 p228)(includes o73 p237)

(waiting o74)
(includes o74 p14)(includes o74 p26)(includes o74 p46)(includes o74 p174)(includes o74 p183)(includes o74 p220)(includes o74 p229)(includes o74 p237)

(waiting o75)
(includes o75 p8)(includes o75 p37)(includes o75 p112)(includes o75 p136)(includes o75 p162)(includes o75 p189)(includes o75 p222)(includes o75 p253)(includes o75 p255)(includes o75 p263)

(waiting o76)
(includes o76 p34)(includes o76 p73)(includes o76 p151)(includes o76 p160)(includes o76 p191)(includes o76 p196)(includes o76 p241)(includes o76 p246)(includes o76 p262)

(waiting o77)
(includes o77 p11)(includes o77 p141)(includes o77 p168)(includes o77 p201)(includes o77 p227)

(waiting o78)
(includes o78 p23)(includes o78 p67)(includes o78 p97)(includes o78 p173)(includes o78 p267)

(waiting o79)
(includes o79 p137)(includes o79 p174)(includes o79 p190)(includes o79 p226)(includes o79 p244)

(waiting o80)
(includes o80 p35)(includes o80 p69)(includes o80 p87)(includes o80 p93)(includes o80 p98)(includes o80 p110)(includes o80 p117)(includes o80 p118)(includes o80 p127)(includes o80 p130)(includes o80 p165)

(waiting o81)
(includes o81 p110)(includes o81 p125)(includes o81 p130)(includes o81 p160)(includes o81 p173)(includes o81 p211)

(waiting o82)
(includes o82 p110)(includes o82 p111)(includes o82 p127)(includes o82 p166)(includes o82 p203)(includes o82 p247)

(waiting o83)
(includes o83 p157)(includes o83 p208)(includes o83 p230)(includes o83 p249)

(waiting o84)
(includes o84 p23)(includes o84 p35)(includes o84 p123)(includes o84 p133)(includes o84 p229)(includes o84 p230)(includes o84 p255)(includes o84 p258)

(waiting o85)
(includes o85 p27)(includes o85 p47)(includes o85 p103)(includes o85 p180)(includes o85 p215)(includes o85 p246)

(waiting o86)
(includes o86 p7)(includes o86 p30)(includes o86 p39)(includes o86 p102)(includes o86 p104)(includes o86 p241)

(waiting o87)
(includes o87 p18)(includes o87 p37)(includes o87 p54)(includes o87 p173)(includes o87 p178)(includes o87 p187)(includes o87 p206)

(waiting o88)
(includes o88 p11)(includes o88 p64)(includes o88 p80)(includes o88 p93)(includes o88 p124)(includes o88 p146)(includes o88 p148)(includes o88 p204)(includes o88 p269)

(waiting o89)
(includes o89 p4)(includes o89 p47)(includes o89 p96)(includes o89 p99)(includes o89 p148)(includes o89 p210)(includes o89 p258)

(waiting o90)
(includes o90 p7)(includes o90 p43)(includes o90 p205)(includes o90 p253)

(waiting o91)
(includes o91 p33)(includes o91 p64)(includes o91 p68)(includes o91 p156)(includes o91 p207)(includes o91 p233)(includes o91 p242)

(waiting o92)
(includes o92 p29)(includes o92 p70)(includes o92 p77)(includes o92 p98)(includes o92 p121)(includes o92 p151)(includes o92 p217)(includes o92 p226)(includes o92 p234)(includes o92 p236)(includes o92 p266)

(waiting o93)
(includes o93 p21)(includes o93 p168)(includes o93 p177)(includes o93 p266)

(waiting o94)
(includes o94 p3)(includes o94 p22)(includes o94 p98)(includes o94 p103)(includes o94 p235)

(waiting o95)
(includes o95 p12)(includes o95 p96)(includes o95 p111)(includes o95 p122)(includes o95 p133)(includes o95 p140)(includes o95 p166)(includes o95 p168)

(waiting o96)
(includes o96 p16)(includes o96 p45)(includes o96 p63)(includes o96 p233)(includes o96 p240)(includes o96 p241)

(waiting o97)
(includes o97 p42)(includes o97 p86)(includes o97 p102)(includes o97 p128)(includes o97 p158)(includes o97 p224)(includes o97 p228)(includes o97 p242)(includes o97 p269)

(waiting o98)
(includes o98 p18)(includes o98 p43)(includes o98 p53)(includes o98 p181)

(waiting o99)
(includes o99 p34)(includes o99 p105)(includes o99 p113)(includes o99 p116)(includes o99 p126)(includes o99 p243)(includes o99 p246)

(waiting o100)
(includes o100 p14)(includes o100 p22)(includes o100 p37)(includes o100 p76)(includes o100 p175)(includes o100 p183)(includes o100 p213)(includes o100 p222)

(waiting o101)
(includes o101 p147)(includes o101 p157)(includes o101 p203)

(waiting o102)
(includes o102 p94)(includes o102 p166)(includes o102 p174)(includes o102 p235)

(waiting o103)
(includes o103 p77)(includes o103 p137)(includes o103 p179)(includes o103 p185)(includes o103 p186)(includes o103 p204)

(waiting o104)
(includes o104 p26)(includes o104 p67)(includes o104 p83)(includes o104 p105)(includes o104 p201)

(waiting o105)
(includes o105 p19)(includes o105 p43)(includes o105 p47)(includes o105 p49)(includes o105 p199)(includes o105 p213)

(waiting o106)
(includes o106 p13)(includes o106 p46)(includes o106 p64)(includes o106 p68)

(waiting o107)
(includes o107 p1)(includes o107 p25)(includes o107 p38)(includes o107 p111)(includes o107 p151)(includes o107 p186)(includes o107 p226)

(waiting o108)
(includes o108 p4)(includes o108 p8)(includes o108 p9)(includes o108 p26)(includes o108 p63)(includes o108 p169)(includes o108 p231)(includes o108 p242)

(waiting o109)
(includes o109 p62)(includes o109 p177)(includes o109 p207)

(waiting o110)
(includes o110 p26)(includes o110 p76)(includes o110 p79)(includes o110 p247)(includes o110 p253)

(waiting o111)
(includes o111 p123)(includes o111 p126)(includes o111 p151)(includes o111 p201)(includes o111 p207)

(waiting o112)
(includes o112 p12)(includes o112 p22)(includes o112 p58)(includes o112 p93)(includes o112 p108)(includes o112 p133)(includes o112 p147)(includes o112 p235)

(waiting o113)
(includes o113 p90)(includes o113 p92)(includes o113 p131)(includes o113 p188)(includes o113 p201)(includes o113 p211)(includes o113 p224)

(waiting o114)
(includes o114 p41)(includes o114 p73)

(waiting o115)
(includes o115 p1)(includes o115 p50)(includes o115 p51)(includes o115 p62)(includes o115 p106)(includes o115 p196)(includes o115 p218)

(waiting o116)
(includes o116 p2)(includes o116 p6)(includes o116 p10)(includes o116 p143)(includes o116 p180)(includes o116 p184)(includes o116 p238)

(waiting o117)
(includes o117 p32)(includes o117 p69)(includes o117 p116)(includes o117 p139)(includes o117 p241)(includes o117 p243)(includes o117 p251)

(waiting o118)
(includes o118 p36)(includes o118 p42)(includes o118 p92)(includes o118 p107)(includes o118 p111)(includes o118 p137)(includes o118 p150)(includes o118 p153)(includes o118 p218)

(waiting o119)
(includes o119 p59)(includes o119 p74)(includes o119 p80)(includes o119 p189)(includes o119 p216)(includes o119 p268)

(waiting o120)
(includes o120 p74)(includes o120 p142)(includes o120 p153)(includes o120 p187)(includes o120 p206)

(waiting o121)
(includes o121 p103)(includes o121 p139)(includes o121 p148)(includes o121 p151)(includes o121 p178)(includes o121 p189)(includes o121 p220)(includes o121 p235)(includes o121 p245)(includes o121 p270)

(waiting o122)
(includes o122 p22)(includes o122 p121)(includes o122 p169)(includes o122 p257)

(waiting o123)
(includes o123 p165)(includes o123 p183)(includes o123 p195)(includes o123 p198)(includes o123 p223)(includes o123 p258)

(waiting o124)
(includes o124 p58)(includes o124 p128)(includes o124 p147)(includes o124 p224)

(waiting o125)
(includes o125 p74)(includes o125 p198)(includes o125 p199)(includes o125 p230)(includes o125 p255)

(waiting o126)
(includes o126 p60)(includes o126 p67)(includes o126 p164)(includes o126 p219)(includes o126 p244)

(waiting o127)
(includes o127 p46)(includes o127 p54)(includes o127 p176)(includes o127 p186)(includes o127 p191)(includes o127 p201)(includes o127 p240)(includes o127 p251)

(waiting o128)
(includes o128 p17)(includes o128 p54)(includes o128 p131)(includes o128 p137)(includes o128 p143)(includes o128 p164)(includes o128 p211)

(waiting o129)
(includes o129 p44)(includes o129 p122)(includes o129 p185)(includes o129 p238)(includes o129 p242)(includes o129 p270)

(waiting o130)
(includes o130 p33)(includes o130 p42)(includes o130 p61)(includes o130 p83)(includes o130 p96)(includes o130 p105)(includes o130 p127)(includes o130 p132)(includes o130 p171)(includes o130 p213)(includes o130 p232)(includes o130 p258)

(waiting o131)
(includes o131 p3)(includes o131 p37)(includes o131 p46)(includes o131 p66)(includes o131 p81)(includes o131 p120)

(waiting o132)
(includes o132 p36)(includes o132 p86)(includes o132 p142)(includes o132 p150)(includes o132 p162)(includes o132 p184)(includes o132 p209)(includes o132 p258)

(waiting o133)
(includes o133 p106)(includes o133 p132)(includes o133 p177)(includes o133 p181)(includes o133 p198)(includes o133 p202)(includes o133 p237)(includes o133 p258)

(waiting o134)
(includes o134 p61)(includes o134 p125)(includes o134 p141)(includes o134 p153)(includes o134 p168)(includes o134 p175)(includes o134 p202)(includes o134 p228)(includes o134 p248)(includes o134 p253)(includes o134 p260)

(waiting o135)
(includes o135 p99)(includes o135 p109)(includes o135 p111)(includes o135 p114)(includes o135 p132)(includes o135 p217)

(waiting o136)
(includes o136 p71)(includes o136 p97)(includes o136 p123)(includes o136 p127)(includes o136 p128)(includes o136 p179)(includes o136 p196)(includes o136 p206)(includes o136 p211)(includes o136 p243)

(waiting o137)
(includes o137 p62)(includes o137 p122)(includes o137 p133)(includes o137 p175)(includes o137 p200)(includes o137 p231)

(waiting o138)
(includes o138 p132)(includes o138 p199)(includes o138 p265)

(waiting o139)
(includes o139 p51)(includes o139 p62)(includes o139 p148)(includes o139 p213)(includes o139 p235)

(waiting o140)
(includes o140 p57)(includes o140 p87)(includes o140 p135)(includes o140 p231)

(waiting o141)
(includes o141 p11)(includes o141 p46)(includes o141 p70)(includes o141 p115)(includes o141 p122)(includes o141 p134)(includes o141 p147)(includes o141 p161)(includes o141 p235)(includes o141 p253)(includes o141 p267)

(waiting o142)
(includes o142 p34)(includes o142 p45)(includes o142 p94)(includes o142 p242)

(waiting o143)
(includes o143 p49)(includes o143 p91)(includes o143 p128)(includes o143 p187)(includes o143 p210)(includes o143 p252)

(waiting o144)
(includes o144 p25)(includes o144 p50)(includes o144 p123)(includes o144 p166)(includes o144 p176)(includes o144 p263)

(waiting o145)
(includes o145 p2)(includes o145 p52)(includes o145 p97)(includes o145 p102)(includes o145 p110)(includes o145 p158)(includes o145 p160)

(waiting o146)
(includes o146 p77)(includes o146 p97)(includes o146 p168)(includes o146 p177)(includes o146 p226)(includes o146 p231)(includes o146 p235)

(waiting o147)
(includes o147 p4)(includes o147 p6)(includes o147 p33)(includes o147 p60)(includes o147 p74)(includes o147 p111)(includes o147 p122)(includes o147 p184)(includes o147 p196)(includes o147 p210)

(waiting o148)
(includes o148 p105)(includes o148 p118)(includes o148 p156)(includes o148 p213)(includes o148 p219)(includes o148 p235)(includes o148 p252)(includes o148 p257)

(waiting o149)
(includes o149 p33)(includes o149 p54)(includes o149 p72)(includes o149 p99)(includes o149 p206)(includes o149 p228)(includes o149 p244)(includes o149 p269)

(waiting o150)
(includes o150 p18)(includes o150 p174)(includes o150 p193)(includes o150 p205)

(waiting o151)
(includes o151 p36)(includes o151 p57)(includes o151 p107)(includes o151 p137)(includes o151 p167)(includes o151 p172)(includes o151 p197)

(waiting o152)
(includes o152 p31)(includes o152 p38)(includes o152 p71)(includes o152 p169)

(waiting o153)
(includes o153 p25)(includes o153 p61)(includes o153 p67)(includes o153 p97)(includes o153 p198)(includes o153 p223)(includes o153 p247)

(waiting o154)
(includes o154 p24)(includes o154 p33)(includes o154 p59)(includes o154 p65)(includes o154 p134)(includes o154 p178)(includes o154 p242)(includes o154 p252)

(waiting o155)
(includes o155 p74)(includes o155 p138)(includes o155 p174)(includes o155 p207)(includes o155 p215)(includes o155 p244)(includes o155 p250)(includes o155 p264)

(waiting o156)
(includes o156 p49)(includes o156 p84)(includes o156 p112)(includes o156 p119)(includes o156 p144)(includes o156 p206)(includes o156 p222)(includes o156 p231)(includes o156 p249)(includes o156 p261)

(waiting o157)
(includes o157 p106)(includes o157 p162)(includes o157 p217)(includes o157 p244)(includes o157 p248)

(waiting o158)
(includes o158 p82)(includes o158 p93)(includes o158 p111)(includes o158 p116)(includes o158 p142)(includes o158 p180)(includes o158 p252)

(waiting o159)
(includes o159 p73)(includes o159 p96)(includes o159 p213)

(waiting o160)
(includes o160 p108)(includes o160 p116)(includes o160 p139)(includes o160 p145)

(waiting o161)
(includes o161 p28)(includes o161 p56)(includes o161 p142)(includes o161 p192)(includes o161 p195)(includes o161 p237)(includes o161 p253)(includes o161 p264)(includes o161 p270)

(waiting o162)
(includes o162 p70)(includes o162 p96)(includes o162 p99)(includes o162 p162)(includes o162 p175)(includes o162 p211)(includes o162 p245)(includes o162 p264)

(waiting o163)
(includes o163 p35)(includes o163 p80)(includes o163 p106)(includes o163 p124)(includes o163 p126)(includes o163 p189)(includes o163 p232)(includes o163 p248)

(waiting o164)
(includes o164 p2)(includes o164 p23)(includes o164 p40)(includes o164 p52)(includes o164 p55)(includes o164 p79)(includes o164 p128)(includes o164 p161)(includes o164 p191)(includes o164 p217)

(waiting o165)
(includes o165 p22)(includes o165 p42)(includes o165 p52)(includes o165 p125)(includes o165 p127)(includes o165 p135)(includes o165 p148)(includes o165 p172)(includes o165 p215)(includes o165 p248)(includes o165 p254)(includes o165 p264)

(waiting o166)
(includes o166 p89)(includes o166 p105)(includes o166 p183)(includes o166 p221)(includes o166 p236)

(waiting o167)
(includes o167 p16)(includes o167 p79)(includes o167 p103)(includes o167 p112)(includes o167 p128)(includes o167 p145)(includes o167 p250)

(waiting o168)
(includes o168 p28)(includes o168 p37)(includes o168 p93)(includes o168 p189)(includes o168 p226)

(waiting o169)
(includes o169 p3)(includes o169 p54)(includes o169 p122)(includes o169 p174)(includes o169 p186)(includes o169 p260)(includes o169 p268)

(waiting o170)
(includes o170 p14)(includes o170 p55)(includes o170 p74)(includes o170 p77)(includes o170 p85)(includes o170 p150)(includes o170 p168)(includes o170 p197)(includes o170 p219)(includes o170 p262)

(waiting o171)
(includes o171 p88)

(waiting o172)
(includes o172 p35)(includes o172 p46)(includes o172 p91)(includes o172 p107)(includes o172 p118)(includes o172 p173)(includes o172 p207)(includes o172 p237)(includes o172 p250)

(waiting o173)
(includes o173 p53)(includes o173 p144)(includes o173 p159)(includes o173 p223)

(waiting o174)
(includes o174 p21)(includes o174 p40)(includes o174 p59)(includes o174 p102)(includes o174 p140)(includes o174 p177)(includes o174 p212)

(waiting o175)
(includes o175 p45)(includes o175 p53)(includes o175 p64)(includes o175 p71)(includes o175 p83)(includes o175 p165)(includes o175 p248)(includes o175 p254)

(waiting o176)
(includes o176 p175)

(waiting o177)
(includes o177 p4)(includes o177 p11)(includes o177 p48)(includes o177 p221)

(waiting o178)
(includes o178 p54)(includes o178 p71)(includes o178 p170)(includes o178 p215)(includes o178 p253)

(waiting o179)
(includes o179 p5)(includes o179 p59)(includes o179 p83)(includes o179 p105)(includes o179 p123)(includes o179 p136)(includes o179 p137)(includes o179 p151)(includes o179 p164)(includes o179 p183)(includes o179 p187)(includes o179 p216)(includes o179 p250)

(waiting o180)
(includes o180 p26)(includes o180 p79)(includes o180 p82)(includes o180 p128)(includes o180 p175)(includes o180 p177)(includes o180 p183)(includes o180 p208)(includes o180 p232)(includes o180 p259)

(waiting o181)
(includes o181 p25)(includes o181 p27)(includes o181 p58)(includes o181 p88)(includes o181 p97)(includes o181 p191)(includes o181 p262)

(waiting o182)
(includes o182 p16)(includes o182 p48)(includes o182 p67)(includes o182 p212)(includes o182 p226)(includes o182 p256)

(waiting o183)
(includes o183 p7)(includes o183 p110)(includes o183 p113)(includes o183 p160)(includes o183 p196)

(waiting o184)
(includes o184 p117)

(waiting o185)
(includes o185 p22)(includes o185 p31)(includes o185 p79)(includes o185 p132)(includes o185 p136)(includes o185 p143)(includes o185 p162)(includes o185 p250)

(waiting o186)
(includes o186 p118)(includes o186 p145)(includes o186 p156)(includes o186 p177)(includes o186 p255)(includes o186 p267)

(waiting o187)
(includes o187 p2)(includes o187 p47)(includes o187 p92)(includes o187 p96)(includes o187 p159)(includes o187 p209)

(waiting o188)
(includes o188 p1)(includes o188 p82)(includes o188 p129)(includes o188 p150)(includes o188 p160)(includes o188 p175)(includes o188 p192)(includes o188 p267)

(waiting o189)
(includes o189 p64)(includes o189 p102)(includes o189 p109)(includes o189 p131)(includes o189 p205)(includes o189 p217)(includes o189 p228)(includes o189 p251)

(waiting o190)
(includes o190 p35)(includes o190 p41)(includes o190 p67)(includes o190 p84)(includes o190 p98)(includes o190 p150)(includes o190 p174)(includes o190 p214)(includes o190 p221)(includes o190 p230)(includes o190 p236)(includes o190 p262)

(waiting o191)
(includes o191 p51)(includes o191 p60)(includes o191 p146)(includes o191 p160)(includes o191 p165)(includes o191 p247)

(waiting o192)
(includes o192 p116)(includes o192 p215)(includes o192 p240)(includes o192 p247)

(waiting o193)
(includes o193 p7)(includes o193 p36)(includes o193 p62)(includes o193 p86)(includes o193 p112)(includes o193 p126)(includes o193 p201)(includes o193 p206)(includes o193 p218)(includes o193 p226)

(waiting o194)
(includes o194 p30)(includes o194 p140)(includes o194 p166)(includes o194 p180)(includes o194 p222)

(waiting o195)
(includes o195 p26)(includes o195 p40)(includes o195 p69)(includes o195 p169)(includes o195 p246)

(waiting o196)
(includes o196 p7)(includes o196 p94)(includes o196 p118)(includes o196 p121)(includes o196 p143)(includes o196 p167)(includes o196 p207)(includes o196 p229)(includes o196 p231)(includes o196 p239)

(waiting o197)
(includes o197 p47)(includes o197 p77)(includes o197 p113)(includes o197 p118)(includes o197 p132)(includes o197 p164)(includes o197 p226)(includes o197 p264)(includes o197 p267)

(waiting o198)
(includes o198 p7)(includes o198 p15)(includes o198 p39)(includes o198 p101)(includes o198 p145)(includes o198 p174)(includes o198 p205)(includes o198 p228)

(waiting o199)
(includes o199 p8)(includes o199 p107)(includes o199 p110)(includes o199 p138)(includes o199 p166)(includes o199 p208)(includes o199 p235)

(waiting o200)
(includes o200 p21)(includes o200 p52)(includes o200 p87)(includes o200 p92)(includes o200 p94)(includes o200 p132)(includes o200 p165)(includes o200 p242)

(waiting o201)
(includes o201 p27)(includes o201 p40)(includes o201 p53)(includes o201 p60)(includes o201 p66)(includes o201 p119)(includes o201 p123)(includes o201 p170)(includes o201 p202)(includes o201 p227)

(waiting o202)
(includes o202 p27)(includes o202 p61)(includes o202 p137)(includes o202 p150)(includes o202 p165)(includes o202 p171)(includes o202 p232)(includes o202 p251)(includes o202 p262)

(waiting o203)
(includes o203 p13)(includes o203 p110)(includes o203 p144)(includes o203 p153)(includes o203 p173)(includes o203 p185)(includes o203 p225)

(waiting o204)
(includes o204 p7)(includes o204 p61)(includes o204 p75)

(waiting o205)
(includes o205 p35)(includes o205 p45)(includes o205 p48)(includes o205 p79)(includes o205 p156)(includes o205 p245)(includes o205 p249)

(waiting o206)
(includes o206 p183)(includes o206 p195)(includes o206 p224)(includes o206 p243)(includes o206 p270)

(waiting o207)
(includes o207 p40)(includes o207 p70)(includes o207 p95)(includes o207 p202)(includes o207 p238)(includes o207 p270)

(waiting o208)
(includes o208 p189)(includes o208 p224)

(waiting o209)
(includes o209 p76)(includes o209 p91)(includes o209 p135)(includes o209 p194)(includes o209 p205)(includes o209 p223)(includes o209 p238)

(waiting o210)
(includes o210 p73)(includes o210 p84)(includes o210 p98)(includes o210 p103)(includes o210 p106)(includes o210 p124)(includes o210 p130)(includes o210 p204)(includes o210 p235)

(waiting o211)
(includes o211 p15)(includes o211 p23)(includes o211 p178)(includes o211 p237)(includes o211 p253)

(waiting o212)
(includes o212 p51)(includes o212 p76)(includes o212 p80)(includes o212 p127)(includes o212 p135)(includes o212 p180)(includes o212 p216)

(waiting o213)
(includes o213 p13)(includes o213 p23)(includes o213 p31)(includes o213 p37)(includes o213 p66)(includes o213 p134)(includes o213 p216)(includes o213 p263)(includes o213 p264)

(waiting o214)
(includes o214 p22)(includes o214 p60)(includes o214 p122)(includes o214 p189)(includes o214 p246)

(waiting o215)
(includes o215 p29)(includes o215 p89)(includes o215 p139)(includes o215 p158)(includes o215 p235)(includes o215 p270)

(waiting o216)
(includes o216 p28)(includes o216 p31)(includes o216 p42)(includes o216 p43)(includes o216 p104)(includes o216 p130)(includes o216 p150)(includes o216 p246)(includes o216 p250)(includes o216 p252)

(waiting o217)
(includes o217 p46)(includes o217 p156)(includes o217 p169)(includes o217 p184)(includes o217 p192)(includes o217 p201)(includes o217 p248)

(waiting o218)
(includes o218 p30)(includes o218 p37)(includes o218 p75)(includes o218 p155)(includes o218 p188)(includes o218 p205)(includes o218 p217)

(waiting o219)
(includes o219 p8)(includes o219 p64)(includes o219 p65)(includes o219 p147)(includes o219 p170)(includes o219 p182)(includes o219 p238)

(waiting o220)
(includes o220 p61)(includes o220 p114)(includes o220 p131)

(waiting o221)
(includes o221 p8)(includes o221 p80)(includes o221 p88)(includes o221 p111)(includes o221 p174)(includes o221 p186)(includes o221 p189)(includes o221 p202)(includes o221 p211)

(waiting o222)
(includes o222 p45)(includes o222 p63)(includes o222 p67)(includes o222 p192)

(waiting o223)
(includes o223 p36)(includes o223 p72)(includes o223 p75)(includes o223 p93)(includes o223 p107)(includes o223 p241)(includes o223 p258)

(waiting o224)
(includes o224 p3)(includes o224 p8)(includes o224 p31)(includes o224 p101)(includes o224 p138)(includes o224 p146)(includes o224 p166)

(waiting o225)
(includes o225 p20)(includes o225 p40)(includes o225 p81)(includes o225 p97)(includes o225 p136)(includes o225 p170)(includes o225 p256)

(waiting o226)
(includes o226 p26)(includes o226 p68)(includes o226 p69)(includes o226 p120)(includes o226 p218)(includes o226 p222)(includes o226 p225)

(waiting o227)
(includes o227 p22)(includes o227 p47)(includes o227 p57)(includes o227 p147)(includes o227 p203)

(waiting o228)
(includes o228 p54)(includes o228 p95)(includes o228 p110)(includes o228 p151)(includes o228 p199)(includes o228 p222)(includes o228 p242)(includes o228 p268)

(waiting o229)
(includes o229 p44)(includes o229 p61)(includes o229 p147)(includes o229 p172)(includes o229 p211)(includes o229 p213)(includes o229 p260)

(waiting o230)
(includes o230 p47)(includes o230 p64)(includes o230 p105)(includes o230 p108)(includes o230 p127)(includes o230 p146)(includes o230 p148)(includes o230 p217)(includes o230 p224)

(waiting o231)
(includes o231 p19)(includes o231 p88)

(waiting o232)
(includes o232 p44)(includes o232 p70)(includes o232 p92)(includes o232 p113)(includes o232 p120)(includes o232 p143)(includes o232 p169)(includes o232 p211)

(waiting o233)
(includes o233 p1)(includes o233 p88)(includes o233 p188)(includes o233 p195)(includes o233 p246)

(waiting o234)
(includes o234 p11)(includes o234 p16)(includes o234 p39)(includes o234 p46)(includes o234 p266)(includes o234 p268)

(waiting o235)
(includes o235 p4)(includes o235 p41)(includes o235 p106)(includes o235 p201)(includes o235 p248)

(waiting o236)
(includes o236 p15)(includes o236 p122)(includes o236 p127)(includes o236 p150)(includes o236 p151)(includes o236 p173)(includes o236 p200)(includes o236 p221)(includes o236 p240)

(waiting o237)
(includes o237 p114)(includes o237 p117)(includes o237 p145)(includes o237 p220)

(waiting o238)
(includes o238 p37)(includes o238 p61)(includes o238 p73)(includes o238 p141)(includes o238 p198)(includes o238 p207)(includes o238 p228)(includes o238 p264)

(waiting o239)
(includes o239 p62)(includes o239 p83)(includes o239 p90)(includes o239 p145)(includes o239 p148)(includes o239 p159)(includes o239 p267)

(waiting o240)
(includes o240 p9)(includes o240 p82)(includes o240 p138)(includes o240 p155)(includes o240 p188)(includes o240 p201)(includes o240 p218)

(waiting o241)
(includes o241 p11)(includes o241 p20)(includes o241 p32)(includes o241 p121)(includes o241 p158)(includes o241 p187)(includes o241 p239)

(waiting o242)
(includes o242 p32)(includes o242 p97)(includes o242 p129)(includes o242 p137)(includes o242 p174)(includes o242 p237)

(waiting o243)
(includes o243 p11)(includes o243 p65)(includes o243 p129)(includes o243 p139)(includes o243 p140)(includes o243 p182)(includes o243 p226)(includes o243 p249)

(waiting o244)
(includes o244 p27)(includes o244 p33)(includes o244 p71)(includes o244 p88)(includes o244 p116)(includes o244 p175)(includes o244 p240)(includes o244 p241)(includes o244 p264)(includes o244 p269)

(waiting o245)
(includes o245 p1)(includes o245 p5)(includes o245 p7)(includes o245 p20)(includes o245 p25)(includes o245 p73)(includes o245 p81)(includes o245 p140)(includes o245 p217)(includes o245 p251)(includes o245 p267)

(waiting o246)
(includes o246 p6)(includes o246 p42)(includes o246 p55)(includes o246 p135)(includes o246 p139)(includes o246 p239)

(waiting o247)
(includes o247 p16)(includes o247 p58)(includes o247 p86)(includes o247 p118)(includes o247 p140)(includes o247 p167)(includes o247 p170)(includes o247 p222)

(waiting o248)
(includes o248 p34)(includes o248 p78)(includes o248 p98)(includes o248 p121)(includes o248 p160)

(waiting o249)
(includes o249 p43)(includes o249 p54)(includes o249 p66)(includes o249 p152)(includes o249 p153)(includes o249 p155)(includes o249 p173)(includes o249 p193)

(waiting o250)
(includes o250 p1)(includes o250 p116)(includes o250 p128)(includes o250 p227)

(waiting o251)
(includes o251 p100)(includes o251 p143)(includes o251 p202)(includes o251 p212)(includes o251 p245)(includes o251 p257)

(waiting o252)
(includes o252 p14)(includes o252 p46)(includes o252 p146)(includes o252 p201)(includes o252 p220)(includes o252 p223)(includes o252 p254)

(waiting o253)
(includes o253 p54)(includes o253 p81)(includes o253 p205)(includes o253 p237)

(waiting o254)
(includes o254 p24)(includes o254 p38)(includes o254 p57)(includes o254 p58)(includes o254 p101)(includes o254 p111)(includes o254 p131)(includes o254 p140)(includes o254 p165)(includes o254 p184)(includes o254 p203)(includes o254 p207)(includes o254 p224)

(waiting o255)
(includes o255 p52)(includes o255 p95)(includes o255 p184)(includes o255 p221)(includes o255 p228)(includes o255 p230)(includes o255 p233)

(waiting o256)
(includes o256 p67)(includes o256 p94)(includes o256 p129)(includes o256 p165)(includes o256 p181)

(waiting o257)
(includes o257 p14)(includes o257 p69)(includes o257 p135)(includes o257 p154)(includes o257 p166)

(waiting o258)
(includes o258 p21)(includes o258 p143)(includes o258 p185)

(waiting o259)
(includes o259 p39)(includes o259 p78)(includes o259 p104)(includes o259 p116)(includes o259 p167)(includes o259 p211)(includes o259 p227)(includes o259 p246)(includes o259 p247)

(waiting o260)
(includes o260 p31)(includes o260 p50)(includes o260 p139)

(waiting o261)
(includes o261 p44)(includes o261 p124)(includes o261 p130)(includes o261 p160)(includes o261 p211)

(waiting o262)
(includes o262 p29)(includes o262 p66)(includes o262 p250)

(waiting o263)
(includes o263 p1)(includes o263 p33)(includes o263 p35)(includes o263 p61)(includes o263 p102)(includes o263 p124)(includes o263 p174)

(waiting o264)
(includes o264 p60)(includes o264 p115)(includes o264 p173)(includes o264 p181)(includes o264 p204)

(waiting o265)
(includes o265 p59)(includes o265 p143)(includes o265 p199)(includes o265 p213)(includes o265 p238)(includes o265 p242)(includes o265 p264)

(waiting o266)
(includes o266 p23)(includes o266 p47)(includes o266 p66)(includes o266 p108)(includes o266 p109)(includes o266 p114)(includes o266 p146)(includes o266 p168)(includes o266 p194)(includes o266 p253)(includes o266 p264)(includes o266 p270)

(waiting o267)
(includes o267 p66)(includes o267 p155)(includes o267 p176)(includes o267 p198)(includes o267 p253)

(waiting o268)
(includes o268 p62)(includes o268 p190)(includes o268 p259)

(waiting o269)
(includes o269 p98)(includes o269 p146)

(waiting o270)
(includes o270 p147)(includes o270 p251)

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
))

(:metric minimize (total-cost))

)

