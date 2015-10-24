(define (problem os-sequencedstrips-p290_1)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210 n211 n212 n213 n214 n215 n216 n217 n218 n219 n220 n221 n222 n223 n224 n225 n226 n227 n228 n229 n230 n231 n232 n233 n234 n235 n236 n237 n238 n239 n240 n241 n242 n243 n244 n245 n246 n247 n248 n249 n250 n251 n252 n253 n254 n255 n256 n257 n258 n259 n260 n261 n262 n263 n264 n265 n266 n267 n268 n269 n270 n271 n272 n273 n274 n275 n276 n277 n278 n279 n280 n281 n282 n283 n284 n285 n286 n287 n288 n289 n290  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) (next-count n210 n211) (next-count n211 n212) (next-count n212 n213) (next-count n213 n214) (next-count n214 n215) (next-count n215 n216) (next-count n216 n217) (next-count n217 n218) (next-count n218 n219) (next-count n219 n220) (next-count n220 n221) (next-count n221 n222) (next-count n222 n223) (next-count n223 n224) (next-count n224 n225) (next-count n225 n226) (next-count n226 n227) (next-count n227 n228) (next-count n228 n229) (next-count n229 n230) (next-count n230 n231) (next-count n231 n232) (next-count n232 n233) (next-count n233 n234) (next-count n234 n235) (next-count n235 n236) (next-count n236 n237) (next-count n237 n238) (next-count n238 n239) (next-count n239 n240) (next-count n240 n241) (next-count n241 n242) (next-count n242 n243) (next-count n243 n244) (next-count n244 n245) (next-count n245 n246) (next-count n246 n247) (next-count n247 n248) (next-count n248 n249) (next-count n249 n250) (next-count n250 n251) (next-count n251 n252) (next-count n252 n253) (next-count n253 n254) (next-count n254 n255) (next-count n255 n256) (next-count n256 n257) (next-count n257 n258) (next-count n258 n259) (next-count n259 n260) (next-count n260 n261) (next-count n261 n262) (next-count n262 n263) (next-count n263 n264) (next-count n264 n265) (next-count n265 n266) (next-count n266 n267) (next-count n267 n268) (next-count n268 n269) (next-count n269 n270) (next-count n270 n271) (next-count n271 n272) (next-count n272 n273) (next-count n273 n274) (next-count n274 n275) (next-count n275 n276) (next-count n276 n277) (next-count n277 n278) (next-count n278 n279) (next-count n279 n280) (next-count n280 n281) (next-count n281 n282) (next-count n282 n283) (next-count n283 n284) (next-count n284 n285) (next-count n285 n286) (next-count n286 n287) (next-count n287 n288) (next-count n288 n289) (next-count n289 n290) 
(stacks-avail n0)

(waiting o1)
(includes o1 p45)(includes o1 p57)(includes o1 p73)(includes o1 p146)(includes o1 p166)(includes o1 p203)(includes o1 p248)(includes o1 p287)

(waiting o2)
(includes o2 p22)(includes o2 p30)(includes o2 p41)(includes o2 p44)(includes o2 p79)(includes o2 p158)(includes o2 p267)

(waiting o3)
(includes o3 p9)(includes o3 p194)

(waiting o4)
(includes o4 p6)(includes o4 p41)(includes o4 p133)(includes o4 p218)(includes o4 p228)(includes o4 p242)(includes o4 p269)

(waiting o5)
(includes o5 p21)(includes o5 p115)

(waiting o6)
(includes o6 p25)(includes o6 p62)(includes o6 p127)(includes o6 p142)(includes o6 p171)(includes o6 p201)(includes o6 p250)(includes o6 p283)

(waiting o7)
(includes o7 p21)(includes o7 p53)(includes o7 p103)(includes o7 p108)(includes o7 p124)(includes o7 p158)

(waiting o8)
(includes o8 p69)(includes o8 p233)(includes o8 p279)

(waiting o9)
(includes o9 p81)(includes o9 p144)(includes o9 p149)(includes o9 p190)(includes o9 p199)(includes o9 p246)(includes o9 p259)

(waiting o10)
(includes o10 p10)(includes o10 p83)(includes o10 p101)(includes o10 p116)(includes o10 p131)(includes o10 p176)(includes o10 p259)

(waiting o11)
(includes o11 p58)(includes o11 p62)(includes o11 p140)(includes o11 p143)(includes o11 p238)(includes o11 p240)

(waiting o12)
(includes o12 p32)(includes o12 p155)(includes o12 p264)

(waiting o13)
(includes o13 p90)(includes o13 p131)(includes o13 p172)(includes o13 p180)(includes o13 p184)(includes o13 p228)(includes o13 p266)

(waiting o14)
(includes o14 p16)(includes o14 p73)(includes o14 p165)

(waiting o15)
(includes o15 p118)(includes o15 p152)(includes o15 p202)(includes o15 p224)(includes o15 p242)(includes o15 p272)

(waiting o16)
(includes o16 p1)(includes o16 p67)(includes o16 p189)(includes o16 p194)(includes o16 p214)(includes o16 p273)(includes o16 p277)

(waiting o17)
(includes o17 p8)(includes o17 p94)(includes o17 p122)(includes o17 p155)(includes o17 p257)(includes o17 p279)(includes o17 p281)

(waiting o18)
(includes o18 p10)(includes o18 p28)(includes o18 p65)(includes o18 p131)(includes o18 p177)(includes o18 p262)

(waiting o19)
(includes o19 p35)(includes o19 p155)(includes o19 p161)(includes o19 p231)

(waiting o20)
(includes o20 p85)(includes o20 p88)(includes o20 p98)(includes o20 p123)(includes o20 p218)(includes o20 p262)

(waiting o21)
(includes o21 p8)(includes o21 p78)(includes o21 p151)(includes o21 p191)(includes o21 p203)(includes o21 p248)(includes o21 p262)(includes o21 p270)

(waiting o22)
(includes o22 p27)(includes o22 p30)(includes o22 p46)(includes o22 p109)(includes o22 p148)(includes o22 p181)(includes o22 p259)(includes o22 p280)

(waiting o23)
(includes o23 p2)(includes o23 p118)(includes o23 p136)(includes o23 p226)(includes o23 p261)(includes o23 p270)

(waiting o24)
(includes o24 p32)(includes o24 p156)(includes o24 p202)(includes o24 p210)(includes o24 p234)(includes o24 p243)(includes o24 p283)

(waiting o25)
(includes o25 p4)(includes o25 p27)(includes o25 p125)(includes o25 p258)

(waiting o26)
(includes o26 p40)(includes o26 p236)

(waiting o27)
(includes o27 p65)(includes o27 p95)(includes o27 p98)(includes o27 p168)(includes o27 p173)(includes o27 p208)(includes o27 p252)

(waiting o28)
(includes o28 p2)(includes o28 p67)(includes o28 p148)(includes o28 p153)(includes o28 p158)

(waiting o29)
(includes o29 p9)(includes o29 p142)(includes o29 p149)(includes o29 p222)(includes o29 p234)

(waiting o30)
(includes o30 p18)(includes o30 p33)(includes o30 p63)(includes o30 p171)(includes o30 p262)

(waiting o31)
(includes o31 p69)(includes o31 p96)(includes o31 p190)(includes o31 p196)(includes o31 p265)(includes o31 p274)

(waiting o32)
(includes o32 p62)(includes o32 p63)(includes o32 p105)(includes o32 p112)(includes o32 p129)(includes o32 p147)(includes o32 p170)(includes o32 p183)(includes o32 p205)(includes o32 p228)(includes o32 p269)(includes o32 p281)

(waiting o33)
(includes o33 p21)(includes o33 p48)(includes o33 p156)(includes o33 p194)(includes o33 p228)(includes o33 p263)

(waiting o34)
(includes o34 p2)(includes o34 p34)(includes o34 p37)(includes o34 p118)(includes o34 p222)(includes o34 p227)(includes o34 p285)

(waiting o35)
(includes o35 p23)(includes o35 p28)(includes o35 p63)(includes o35 p116)(includes o35 p254)(includes o35 p271)(includes o35 p278)

(waiting o36)
(includes o36 p22)(includes o36 p38)(includes o36 p63)(includes o36 p99)(includes o36 p247)

(waiting o37)
(includes o37 p29)(includes o37 p36)(includes o37 p88)(includes o37 p215)(includes o37 p222)(includes o37 p264)(includes o37 p267)

(waiting o38)
(includes o38 p19)(includes o38 p45)(includes o38 p120)(includes o38 p158)(includes o38 p186)(includes o38 p215)(includes o38 p246)

(waiting o39)
(includes o39 p123)(includes o39 p131)(includes o39 p132)(includes o39 p135)

(waiting o40)
(includes o40 p88)(includes o40 p119)(includes o40 p137)(includes o40 p242)(includes o40 p249)(includes o40 p267)(includes o40 p273)

(waiting o41)
(includes o41 p14)(includes o41 p22)(includes o41 p150)(includes o41 p156)(includes o41 p161)(includes o41 p175)(includes o41 p194)(includes o41 p201)(includes o41 p226)(includes o41 p258)(includes o41 p290)

(waiting o42)
(includes o42 p28)(includes o42 p54)(includes o42 p90)(includes o42 p115)(includes o42 p175)(includes o42 p225)(includes o42 p281)

(waiting o43)
(includes o43 p32)(includes o43 p66)(includes o43 p77)(includes o43 p95)(includes o43 p104)(includes o43 p174)(includes o43 p207)(includes o43 p212)(includes o43 p222)

(waiting o44)
(includes o44 p17)(includes o44 p18)(includes o44 p21)(includes o44 p75)(includes o44 p100)(includes o44 p126)(includes o44 p208)(includes o44 p218)(includes o44 p236)(includes o44 p267)

(waiting o45)
(includes o45 p19)(includes o45 p88)(includes o45 p97)(includes o45 p144)(includes o45 p165)(includes o45 p184)(includes o45 p268)

(waiting o46)
(includes o46 p89)(includes o46 p141)(includes o46 p156)(includes o46 p186)(includes o46 p217)(includes o46 p230)(includes o46 p273)

(waiting o47)
(includes o47 p5)(includes o47 p18)(includes o47 p92)(includes o47 p112)(includes o47 p143)(includes o47 p154)(includes o47 p161)(includes o47 p206)(includes o47 p287)

(waiting o48)
(includes o48 p18)(includes o48 p57)(includes o48 p68)(includes o48 p77)(includes o48 p82)(includes o48 p106)(includes o48 p185)(includes o48 p186)(includes o48 p200)(includes o48 p248)(includes o48 p287)

(waiting o49)
(includes o49 p88)(includes o49 p115)(includes o49 p140)(includes o49 p150)(includes o49 p182)(includes o49 p271)

(waiting o50)
(includes o50 p3)(includes o50 p144)(includes o50 p148)(includes o50 p151)(includes o50 p157)(includes o50 p216)(includes o50 p227)(includes o50 p283)

(waiting o51)
(includes o51 p48)(includes o51 p206)

(waiting o52)
(includes o52 p56)(includes o52 p131)(includes o52 p175)(includes o52 p216)(includes o52 p219)(includes o52 p255)

(waiting o53)
(includes o53 p81)(includes o53 p93)(includes o53 p140)(includes o53 p146)(includes o53 p269)

(waiting o54)
(includes o54 p26)(includes o54 p29)(includes o54 p74)(includes o54 p122)(includes o54 p190)(includes o54 p267)

(waiting o55)
(includes o55 p35)(includes o55 p69)(includes o55 p79)(includes o55 p138)(includes o55 p150)(includes o55 p163)(includes o55 p193)(includes o55 p260)

(waiting o56)
(includes o56 p18)(includes o56 p58)(includes o56 p115)(includes o56 p155)(includes o56 p158)(includes o56 p184)(includes o56 p259)

(waiting o57)
(includes o57 p6)(includes o57 p79)(includes o57 p191)(includes o57 p212)(includes o57 p227)(includes o57 p245)(includes o57 p249)

(waiting o58)
(includes o58 p63)(includes o58 p168)(includes o58 p255)(includes o58 p281)

(waiting o59)
(includes o59 p50)(includes o59 p117)(includes o59 p137)(includes o59 p147)(includes o59 p163)(includes o59 p189)(includes o59 p231)(includes o59 p253)

(waiting o60)
(includes o60 p1)(includes o60 p80)(includes o60 p146)(includes o60 p148)(includes o60 p156)(includes o60 p161)(includes o60 p167)(includes o60 p207)(includes o60 p249)

(waiting o61)
(includes o61 p64)(includes o61 p66)(includes o61 p127)(includes o61 p145)(includes o61 p171)(includes o61 p224)(includes o61 p232)(includes o61 p285)

(waiting o62)
(includes o62 p33)(includes o62 p44)(includes o62 p117)(includes o62 p170)(includes o62 p186)(includes o62 p213)(includes o62 p218)(includes o62 p220)(includes o62 p225)

(waiting o63)
(includes o63 p19)(includes o63 p81)(includes o63 p138)(includes o63 p200)(includes o63 p209)(includes o63 p236)

(waiting o64)
(includes o64 p1)(includes o64 p15)(includes o64 p65)(includes o64 p103)(includes o64 p172)(includes o64 p176)(includes o64 p214)(includes o64 p263)

(waiting o65)
(includes o65 p2)(includes o65 p64)(includes o65 p74)(includes o65 p136)(includes o65 p220)(includes o65 p267)

(waiting o66)
(includes o66 p27)(includes o66 p65)(includes o66 p84)(includes o66 p131)(includes o66 p173)(includes o66 p200)

(waiting o67)
(includes o67 p34)(includes o67 p60)(includes o67 p76)(includes o67 p141)

(waiting o68)
(includes o68 p8)(includes o68 p63)(includes o68 p145)(includes o68 p228)(includes o68 p236)

(waiting o69)
(includes o69 p39)(includes o69 p74)(includes o69 p79)(includes o69 p220)

(waiting o70)
(includes o70 p47)(includes o70 p65)(includes o70 p68)(includes o70 p83)(includes o70 p89)(includes o70 p122)(includes o70 p178)(includes o70 p229)(includes o70 p231)

(waiting o71)
(includes o71 p60)(includes o71 p91)(includes o71 p105)(includes o71 p143)(includes o71 p158)(includes o71 p198)(includes o71 p211)(includes o71 p242)(includes o71 p248)(includes o71 p264)(includes o71 p278)

(waiting o72)
(includes o72 p41)(includes o72 p88)(includes o72 p216)(includes o72 p248)(includes o72 p275)

(waiting o73)
(includes o73 p18)(includes o73 p35)(includes o73 p93)(includes o73 p94)(includes o73 p128)(includes o73 p135)(includes o73 p163)(includes o73 p225)

(waiting o74)
(includes o74 p46)(includes o74 p173)(includes o74 p222)

(waiting o75)
(includes o75 p32)(includes o75 p70)(includes o75 p76)(includes o75 p80)(includes o75 p156)(includes o75 p187)(includes o75 p224)(includes o75 p258)(includes o75 p260)

(waiting o76)
(includes o76 p64)(includes o76 p113)(includes o76 p138)(includes o76 p200)(includes o76 p201)

(waiting o77)
(includes o77 p2)(includes o77 p6)(includes o77 p92)(includes o77 p135)(includes o77 p140)(includes o77 p170)(includes o77 p185)(includes o77 p206)

(waiting o78)
(includes o78 p13)(includes o78 p114)(includes o78 p143)(includes o78 p188)(includes o78 p237)(includes o78 p255)(includes o78 p275)

(waiting o79)
(includes o79 p6)(includes o79 p36)(includes o79 p79)(includes o79 p82)(includes o79 p124)(includes o79 p151)(includes o79 p203)(includes o79 p243)(includes o79 p257)(includes o79 p274)

(waiting o80)
(includes o80 p110)(includes o80 p123)(includes o80 p174)(includes o80 p186)(includes o80 p204)(includes o80 p214)(includes o80 p231)(includes o80 p235)

(waiting o81)
(includes o81 p53)(includes o81 p65)(includes o81 p93)(includes o81 p158)(includes o81 p214)(includes o81 p217)(includes o81 p245)

(waiting o82)
(includes o82 p7)(includes o82 p10)(includes o82 p51)(includes o82 p73)(includes o82 p255)

(waiting o83)
(includes o83 p44)(includes o83 p84)(includes o83 p160)(includes o83 p181)(includes o83 p203)(includes o83 p245)(includes o83 p256)

(waiting o84)
(includes o84 p30)(includes o84 p150)(includes o84 p241)

(waiting o85)
(includes o85 p2)(includes o85 p115)(includes o85 p143)(includes o85 p209)(includes o85 p244)(includes o85 p254)(includes o85 p281)

(waiting o86)
(includes o86 p54)(includes o86 p77)(includes o86 p134)(includes o86 p172)(includes o86 p248)

(waiting o87)
(includes o87 p38)(includes o87 p63)(includes o87 p112)(includes o87 p123)(includes o87 p223)(includes o87 p236)(includes o87 p281)

(waiting o88)
(includes o88 p81)(includes o88 p114)(includes o88 p157)(includes o88 p163)(includes o88 p174)(includes o88 p181)(includes o88 p212)(includes o88 p276)

(waiting o89)
(includes o89 p56)(includes o89 p79)(includes o89 p118)(includes o89 p129)(includes o89 p185)(includes o89 p188)(includes o89 p193)(includes o89 p219)(includes o89 p221)(includes o89 p282)

(waiting o90)
(includes o90 p5)(includes o90 p131)(includes o90 p145)(includes o90 p196)(includes o90 p239)(includes o90 p281)

(waiting o91)
(includes o91 p13)(includes o91 p60)(includes o91 p142)(includes o91 p209)(includes o91 p227)(includes o91 p249)(includes o91 p260)(includes o91 p270)

(waiting o92)
(includes o92 p28)(includes o92 p44)(includes o92 p93)(includes o92 p131)

(waiting o93)
(includes o93 p35)(includes o93 p55)(includes o93 p67)(includes o93 p95)(includes o93 p192)(includes o93 p243)(includes o93 p272)(includes o93 p276)

(waiting o94)
(includes o94 p26)(includes o94 p30)(includes o94 p42)(includes o94 p66)(includes o94 p69)(includes o94 p82)(includes o94 p92)(includes o94 p102)(includes o94 p128)(includes o94 p154)(includes o94 p163)(includes o94 p251)(includes o94 p271)

(waiting o95)
(includes o95 p52)(includes o95 p82)(includes o95 p90)(includes o95 p106)(includes o95 p148)(includes o95 p236)(includes o95 p255)(includes o95 p267)(includes o95 p274)

(waiting o96)
(includes o96 p10)(includes o96 p49)(includes o96 p104)(includes o96 p251)

(waiting o97)
(includes o97 p13)(includes o97 p172)

(waiting o98)
(includes o98 p18)(includes o98 p45)(includes o98 p118)(includes o98 p146)(includes o98 p159)(includes o98 p190)(includes o98 p191)(includes o98 p208)(includes o98 p238)(includes o98 p251)(includes o98 p255)

(waiting o99)
(includes o99 p8)(includes o99 p17)(includes o99 p23)(includes o99 p56)(includes o99 p91)(includes o99 p114)(includes o99 p191)(includes o99 p219)

(waiting o100)
(includes o100 p3)(includes o100 p4)(includes o100 p46)(includes o100 p113)(includes o100 p160)

(waiting o101)
(includes o101 p101)(includes o101 p104)(includes o101 p143)(includes o101 p185)(includes o101 p252)(includes o101 p285)

(waiting o102)
(includes o102 p40)(includes o102 p114)(includes o102 p183)(includes o102 p188)(includes o102 p208)(includes o102 p236)(includes o102 p247)(includes o102 p286)

(waiting o103)
(includes o103 p46)(includes o103 p48)(includes o103 p203)(includes o103 p265)

(waiting o104)
(includes o104 p50)(includes o104 p98)(includes o104 p183)(includes o104 p186)(includes o104 p285)

(waiting o105)
(includes o105 p3)(includes o105 p48)(includes o105 p64)(includes o105 p141)(includes o105 p223)(includes o105 p231)(includes o105 p243)(includes o105 p262)

(waiting o106)
(includes o106 p5)(includes o106 p6)(includes o106 p14)(includes o106 p19)(includes o106 p59)(includes o106 p61)(includes o106 p81)(includes o106 p135)

(waiting o107)
(includes o107 p11)(includes o107 p16)(includes o107 p18)(includes o107 p45)(includes o107 p121)(includes o107 p131)(includes o107 p169)(includes o107 p229)

(waiting o108)
(includes o108 p3)(includes o108 p10)(includes o108 p29)(includes o108 p52)(includes o108 p73)(includes o108 p101)(includes o108 p163)(includes o108 p187)(includes o108 p206)(includes o108 p264)

(waiting o109)
(includes o109 p29)(includes o109 p102)(includes o109 p164)(includes o109 p188)(includes o109 p209)(includes o109 p218)(includes o109 p287)

(waiting o110)
(includes o110 p4)(includes o110 p116)(includes o110 p185)(includes o110 p198)(includes o110 p243)(includes o110 p244)(includes o110 p250)

(waiting o111)
(includes o111 p3)(includes o111 p15)(includes o111 p81)(includes o111 p121)(includes o111 p169)

(waiting o112)
(includes o112 p42)(includes o112 p87)(includes o112 p91)(includes o112 p123)(includes o112 p126)(includes o112 p133)(includes o112 p199)(includes o112 p215)(includes o112 p229)(includes o112 p234)(includes o112 p246)(includes o112 p262)(includes o112 p286)

(waiting o113)
(includes o113 p2)(includes o113 p78)(includes o113 p248)

(waiting o114)
(includes o114 p8)(includes o114 p13)(includes o114 p33)(includes o114 p37)(includes o114 p38)(includes o114 p48)(includes o114 p116)(includes o114 p121)(includes o114 p145)(includes o114 p204)(includes o114 p262)

(waiting o115)
(includes o115 p99)(includes o115 p112)(includes o115 p167)(includes o115 p186)(includes o115 p218)(includes o115 p229)

(waiting o116)
(includes o116 p10)(includes o116 p22)(includes o116 p47)(includes o116 p92)(includes o116 p130)(includes o116 p144)(includes o116 p165)(includes o116 p188)(includes o116 p205)(includes o116 p210)(includes o116 p218)(includes o116 p247)

(waiting o117)
(includes o117 p70)(includes o117 p94)(includes o117 p109)(includes o117 p156)(includes o117 p211)(includes o117 p222)(includes o117 p283)

(waiting o118)
(includes o118 p63)(includes o118 p72)(includes o118 p94)(includes o118 p103)(includes o118 p149)(includes o118 p214)(includes o118 p268)

(waiting o119)
(includes o119 p118)(includes o119 p125)(includes o119 p160)(includes o119 p261)

(waiting o120)
(includes o120 p9)(includes o120 p26)(includes o120 p44)(includes o120 p104)(includes o120 p119)(includes o120 p148)(includes o120 p161)(includes o120 p169)(includes o120 p207)(includes o120 p277)(includes o120 p289)

(waiting o121)
(includes o121 p78)(includes o121 p163)(includes o121 p179)(includes o121 p194)(includes o121 p207)(includes o121 p212)(includes o121 p216)(includes o121 p262)

(waiting o122)
(includes o122 p12)(includes o122 p65)(includes o122 p103)(includes o122 p113)(includes o122 p130)(includes o122 p209)(includes o122 p217)(includes o122 p251)(includes o122 p269)(includes o122 p282)

(waiting o123)
(includes o123 p19)(includes o123 p47)(includes o123 p135)(includes o123 p155)(includes o123 p255)(includes o123 p288)

(waiting o124)
(includes o124 p7)(includes o124 p55)(includes o124 p74)(includes o124 p79)(includes o124 p104)(includes o124 p110)(includes o124 p119)(includes o124 p166)(includes o124 p192)(includes o124 p249)(includes o124 p272)

(waiting o125)
(includes o125 p8)(includes o125 p10)(includes o125 p18)(includes o125 p61)(includes o125 p81)(includes o125 p91)(includes o125 p121)(includes o125 p135)(includes o125 p165)(includes o125 p175)(includes o125 p251)(includes o125 p275)(includes o125 p276)

(waiting o126)
(includes o126 p39)(includes o126 p112)(includes o126 p136)(includes o126 p152)(includes o126 p197)(includes o126 p209)(includes o126 p216)(includes o126 p242)(includes o126 p287)

(waiting o127)
(includes o127 p49)(includes o127 p79)(includes o127 p141)

(waiting o128)
(includes o128 p98)(includes o128 p216)(includes o128 p264)(includes o128 p281)

(waiting o129)
(includes o129 p25)(includes o129 p85)(includes o129 p125)(includes o129 p127)(includes o129 p197)(includes o129 p265)

(waiting o130)
(includes o130 p18)(includes o130 p37)(includes o130 p100)(includes o130 p139)(includes o130 p161)(includes o130 p165)(includes o130 p183)(includes o130 p209)(includes o130 p217)(includes o130 p239)

(waiting o131)
(includes o131 p59)(includes o131 p82)(includes o131 p147)(includes o131 p159)(includes o131 p216)(includes o131 p250)(includes o131 p282)

(waiting o132)
(includes o132 p1)(includes o132 p15)(includes o132 p224)

(waiting o133)
(includes o133 p22)(includes o133 p38)(includes o133 p40)(includes o133 p41)(includes o133 p46)(includes o133 p53)(includes o133 p55)(includes o133 p77)(includes o133 p100)(includes o133 p106)(includes o133 p127)(includes o133 p148)(includes o133 p209)(includes o133 p211)(includes o133 p212)(includes o133 p222)(includes o133 p226)(includes o133 p246)

(waiting o134)
(includes o134 p9)(includes o134 p22)(includes o134 p114)(includes o134 p176)(includes o134 p205)(includes o134 p225)(includes o134 p278)

(waiting o135)
(includes o135 p4)(includes o135 p16)(includes o135 p19)(includes o135 p162)(includes o135 p164)(includes o135 p252)(includes o135 p283)

(waiting o136)
(includes o136 p124)(includes o136 p222)

(waiting o137)
(includes o137 p18)(includes o137 p232)

(waiting o138)
(includes o138 p8)(includes o138 p67)(includes o138 p280)

(waiting o139)
(includes o139 p31)(includes o139 p42)(includes o139 p75)(includes o139 p79)(includes o139 p114)(includes o139 p121)(includes o139 p123)(includes o139 p282)

(waiting o140)
(includes o140 p17)(includes o140 p29)(includes o140 p80)(includes o140 p91)(includes o140 p103)(includes o140 p166)(includes o140 p241)

(waiting o141)
(includes o141 p58)(includes o141 p107)(includes o141 p194)(includes o141 p198)(includes o141 p221)(includes o141 p285)

(waiting o142)
(includes o142 p40)(includes o142 p55)(includes o142 p125)(includes o142 p147)(includes o142 p148)(includes o142 p191)

(waiting o143)
(includes o143 p2)(includes o143 p52)(includes o143 p115)(includes o143 p126)(includes o143 p152)(includes o143 p244)

(waiting o144)
(includes o144 p16)(includes o144 p39)(includes o144 p69)(includes o144 p127)(includes o144 p163)(includes o144 p240)

(waiting o145)
(includes o145 p9)(includes o145 p68)(includes o145 p107)(includes o145 p118)(includes o145 p143)(includes o145 p172)(includes o145 p268)(includes o145 p275)

(waiting o146)
(includes o146 p39)(includes o146 p74)(includes o146 p143)(includes o146 p197)(includes o146 p272)

(waiting o147)
(includes o147 p37)(includes o147 p48)(includes o147 p71)(includes o147 p140)(includes o147 p267)

(waiting o148)
(includes o148 p31)(includes o148 p56)(includes o148 p57)(includes o148 p79)(includes o148 p162)(includes o148 p204)(includes o148 p205)

(waiting o149)
(includes o149 p114)(includes o149 p180)(includes o149 p193)(includes o149 p221)(includes o149 p258)(includes o149 p275)

(waiting o150)
(includes o150 p7)(includes o150 p14)(includes o150 p43)(includes o150 p51)(includes o150 p78)(includes o150 p169)(includes o150 p202)(includes o150 p236)(includes o150 p252)(includes o150 p266)(includes o150 p277)(includes o150 p287)

(waiting o151)
(includes o151 p16)(includes o151 p75)(includes o151 p138)(includes o151 p161)(includes o151 p181)(includes o151 p211)(includes o151 p221)(includes o151 p262)

(waiting o152)
(includes o152 p22)(includes o152 p50)(includes o152 p108)(includes o152 p165)(includes o152 p198)(includes o152 p266)

(waiting o153)
(includes o153 p30)(includes o153 p37)(includes o153 p78)(includes o153 p193)(includes o153 p197)

(waiting o154)
(includes o154 p63)(includes o154 p81)(includes o154 p94)(includes o154 p112)(includes o154 p120)(includes o154 p126)(includes o154 p167)(includes o154 p169)(includes o154 p232)(includes o154 p261)(includes o154 p280)(includes o154 p282)

(waiting o155)
(includes o155 p64)(includes o155 p78)(includes o155 p104)(includes o155 p106)(includes o155 p140)(includes o155 p212)(includes o155 p220)(includes o155 p254)(includes o155 p258)(includes o155 p265)(includes o155 p272)

(waiting o156)
(includes o156 p72)(includes o156 p78)(includes o156 p93)

(waiting o157)
(includes o157 p36)(includes o157 p69)(includes o157 p76)(includes o157 p104)(includes o157 p134)(includes o157 p142)(includes o157 p212)(includes o157 p242)(includes o157 p253)(includes o157 p265)

(waiting o158)
(includes o158 p93)(includes o158 p119)(includes o158 p138)(includes o158 p142)(includes o158 p159)(includes o158 p207)(includes o158 p232)(includes o158 p242)(includes o158 p266)

(waiting o159)
(includes o159 p156)(includes o159 p171)(includes o159 p240)(includes o159 p265)

(waiting o160)
(includes o160 p48)(includes o160 p285)

(waiting o161)
(includes o161 p88)(includes o161 p132)(includes o161 p134)(includes o161 p169)(includes o161 p238)(includes o161 p270)

(waiting o162)
(includes o162 p16)(includes o162 p59)(includes o162 p70)(includes o162 p83)(includes o162 p86)(includes o162 p101)(includes o162 p142)(includes o162 p166)(includes o162 p188)(includes o162 p191)(includes o162 p271)(includes o162 p276)

(waiting o163)
(includes o163 p2)(includes o163 p12)(includes o163 p139)(includes o163 p144)(includes o163 p217)(includes o163 p218)(includes o163 p230)(includes o163 p245)

(waiting o164)
(includes o164 p34)(includes o164 p42)(includes o164 p51)(includes o164 p92)(includes o164 p124)(includes o164 p148)(includes o164 p163)(includes o164 p222)(includes o164 p227)(includes o164 p252)

(waiting o165)
(includes o165 p18)(includes o165 p20)(includes o165 p62)(includes o165 p109)(includes o165 p115)(includes o165 p132)(includes o165 p181)(includes o165 p188)(includes o165 p230)(includes o165 p255)(includes o165 p272)(includes o165 p275)(includes o165 p285)(includes o165 p286)

(waiting o166)
(includes o166 p189)(includes o166 p192)(includes o166 p200)

(waiting o167)
(includes o167 p133)(includes o167 p172)(includes o167 p196)(includes o167 p222)(includes o167 p242)(includes o167 p264)(includes o167 p284)

(waiting o168)
(includes o168 p3)(includes o168 p18)(includes o168 p71)(includes o168 p145)(includes o168 p177)(includes o168 p194)(includes o168 p250)(includes o168 p270)

(waiting o169)
(includes o169 p79)(includes o169 p113)(includes o169 p119)(includes o169 p192)(includes o169 p194)(includes o169 p262)(includes o169 p273)

(waiting o170)
(includes o170 p72)(includes o170 p105)(includes o170 p141)(includes o170 p160)(includes o170 p190)(includes o170 p208)

(waiting o171)
(includes o171 p70)(includes o171 p80)(includes o171 p119)(includes o171 p220)(includes o171 p280)

(waiting o172)
(includes o172 p144)(includes o172 p195)

(waiting o173)
(includes o173 p44)(includes o173 p58)(includes o173 p78)(includes o173 p93)(includes o173 p149)(includes o173 p167)(includes o173 p245)(includes o173 p288)

(waiting o174)
(includes o174 p31)(includes o174 p63)(includes o174 p89)(includes o174 p181)(includes o174 p217)(includes o174 p218)(includes o174 p238)(includes o174 p271)

(waiting o175)
(includes o175 p9)(includes o175 p55)(includes o175 p108)(includes o175 p142)(includes o175 p206)

(waiting o176)
(includes o176 p36)(includes o176 p56)(includes o176 p65)(includes o176 p84)(includes o176 p132)(includes o176 p144)(includes o176 p151)(includes o176 p265)

(waiting o177)
(includes o177 p38)(includes o177 p82)(includes o177 p97)(includes o177 p108)(includes o177 p120)(includes o177 p121)(includes o177 p147)(includes o177 p163)(includes o177 p187)(includes o177 p225)(includes o177 p227)

(waiting o178)
(includes o178 p6)(includes o178 p45)(includes o178 p58)(includes o178 p70)(includes o178 p142)(includes o178 p175)(includes o178 p231)(includes o178 p251)(includes o178 p253)(includes o178 p256)

(waiting o179)
(includes o179 p31)(includes o179 p48)(includes o179 p67)(includes o179 p101)(includes o179 p127)(includes o179 p159)(includes o179 p171)(includes o179 p192)(includes o179 p209)(includes o179 p247)(includes o179 p283)

(waiting o180)
(includes o180 p10)(includes o180 p83)(includes o180 p149)(includes o180 p239)

(waiting o181)
(includes o181 p29)(includes o181 p92)(includes o181 p97)(includes o181 p124)(includes o181 p139)(includes o181 p140)(includes o181 p141)(includes o181 p144)(includes o181 p161)(includes o181 p253)(includes o181 p277)

(waiting o182)
(includes o182 p46)(includes o182 p120)(includes o182 p147)(includes o182 p217)(includes o182 p227)(includes o182 p287)

(waiting o183)
(includes o183 p4)(includes o183 p61)(includes o183 p75)(includes o183 p111)(includes o183 p118)(includes o183 p180)(includes o183 p188)(includes o183 p231)

(waiting o184)
(includes o184 p54)(includes o184 p121)(includes o184 p204)(includes o184 p235)(includes o184 p264)

(waiting o185)
(includes o185 p31)(includes o185 p53)(includes o185 p121)(includes o185 p123)(includes o185 p172)

(waiting o186)
(includes o186 p13)(includes o186 p45)(includes o186 p61)(includes o186 p77)(includes o186 p90)(includes o186 p186)(includes o186 p266)(includes o186 p287)

(waiting o187)
(includes o187 p12)(includes o187 p25)(includes o187 p42)(includes o187 p50)(includes o187 p68)(includes o187 p201)(includes o187 p208)(includes o187 p216)(includes o187 p258)

(waiting o188)
(includes o188 p132)(includes o188 p157)(includes o188 p161)(includes o188 p244)(includes o188 p264)

(waiting o189)
(includes o189 p20)(includes o189 p86)(includes o189 p98)(includes o189 p181)(includes o189 p247)(includes o189 p260)(includes o189 p287)

(waiting o190)
(includes o190 p30)(includes o190 p72)(includes o190 p79)(includes o190 p283)

(waiting o191)
(includes o191 p14)(includes o191 p28)(includes o191 p32)(includes o191 p136)(includes o191 p149)(includes o191 p187)(includes o191 p259)(includes o191 p263)

(waiting o192)
(includes o192 p217)(includes o192 p284)

(waiting o193)
(includes o193 p51)(includes o193 p72)(includes o193 p114)(includes o193 p148)(includes o193 p254)

(waiting o194)
(includes o194 p16)(includes o194 p24)(includes o194 p66)(includes o194 p89)(includes o194 p173)(includes o194 p255)

(waiting o195)
(includes o195 p26)(includes o195 p235)

(waiting o196)
(includes o196 p3)(includes o196 p17)(includes o196 p26)(includes o196 p108)(includes o196 p232)(includes o196 p288)(includes o196 p290)

(waiting o197)
(includes o197 p28)(includes o197 p54)(includes o197 p134)(includes o197 p151)

(waiting o198)
(includes o198 p10)(includes o198 p25)(includes o198 p76)(includes o198 p101)(includes o198 p138)(includes o198 p189)(includes o198 p267)

(waiting o199)
(includes o199 p103)(includes o199 p140)(includes o199 p149)(includes o199 p177)(includes o199 p215)(includes o199 p218)(includes o199 p224)

(waiting o200)
(includes o200 p80)(includes o200 p147)(includes o200 p175)(includes o200 p198)(includes o200 p208)(includes o200 p212)

(waiting o201)
(includes o201 p25)(includes o201 p26)(includes o201 p59)(includes o201 p66)(includes o201 p96)(includes o201 p161)(includes o201 p226)

(waiting o202)
(includes o202 p48)(includes o202 p97)(includes o202 p102)(includes o202 p175)

(waiting o203)
(includes o203 p27)(includes o203 p66)(includes o203 p128)(includes o203 p270)

(waiting o204)
(includes o204 p31)(includes o204 p105)(includes o204 p146)(includes o204 p179)(includes o204 p186)(includes o204 p213)(includes o204 p237)(includes o204 p268)

(waiting o205)
(includes o205 p64)(includes o205 p90)(includes o205 p124)(includes o205 p173)(includes o205 p234)(includes o205 p268)

(waiting o206)
(includes o206 p101)(includes o206 p124)(includes o206 p189)(includes o206 p195)(includes o206 p204)(includes o206 p230)(includes o206 p269)(includes o206 p271)

(waiting o207)
(includes o207 p12)(includes o207 p60)(includes o207 p72)(includes o207 p97)(includes o207 p194)(includes o207 p197)(includes o207 p265)

(waiting o208)
(includes o208 p5)(includes o208 p10)(includes o208 p103)(includes o208 p284)

(waiting o209)
(includes o209 p37)(includes o209 p100)(includes o209 p142)(includes o209 p146)(includes o209 p208)(includes o209 p262)(includes o209 p266)(includes o209 p270)

(waiting o210)
(includes o210 p95)(includes o210 p117)(includes o210 p162)(includes o210 p210)(includes o210 p222)(includes o210 p243)(includes o210 p272)

(waiting o211)
(includes o211 p95)(includes o211 p114)(includes o211 p122)(includes o211 p126)(includes o211 p252)

(waiting o212)
(includes o212 p174)(includes o212 p198)(includes o212 p223)(includes o212 p263)(includes o212 p276)(includes o212 p281)

(waiting o213)
(includes o213 p12)(includes o213 p50)(includes o213 p139)(includes o213 p224)(includes o213 p241)(includes o213 p286)

(waiting o214)
(includes o214 p24)(includes o214 p43)(includes o214 p77)(includes o214 p115)(includes o214 p259)(includes o214 p260)

(waiting o215)
(includes o215 p113)(includes o215 p152)(includes o215 p185)

(waiting o216)
(includes o216 p29)(includes o216 p63)(includes o216 p123)(includes o216 p155)(includes o216 p247)(includes o216 p271)

(waiting o217)
(includes o217 p25)(includes o217 p163)(includes o217 p171)(includes o217 p257)(includes o217 p260)

(waiting o218)
(includes o218 p31)(includes o218 p45)(includes o218 p53)(includes o218 p63)(includes o218 p90)(includes o218 p108)(includes o218 p129)

(waiting o219)
(includes o219 p7)(includes o219 p53)(includes o219 p97)(includes o219 p132)(includes o219 p189)(includes o219 p209)(includes o219 p225)

(waiting o220)
(includes o220 p11)(includes o220 p28)(includes o220 p55)(includes o220 p94)(includes o220 p112)(includes o220 p176)(includes o220 p179)(includes o220 p187)(includes o220 p219)(includes o220 p233)(includes o220 p241)

(waiting o221)
(includes o221 p49)(includes o221 p86)(includes o221 p88)(includes o221 p149)(includes o221 p203)(includes o221 p256)(includes o221 p265)

(waiting o222)
(includes o222 p42)(includes o222 p47)(includes o222 p68)(includes o222 p132)(includes o222 p145)(includes o222 p214)(includes o222 p235)

(waiting o223)
(includes o223 p2)(includes o223 p10)(includes o223 p38)(includes o223 p44)(includes o223 p145)(includes o223 p159)

(waiting o224)
(includes o224 p43)(includes o224 p52)(includes o224 p80)(includes o224 p99)(includes o224 p122)(includes o224 p164)(includes o224 p207)

(waiting o225)
(includes o225 p24)(includes o225 p25)(includes o225 p34)(includes o225 p117)(includes o225 p141)(includes o225 p248)(includes o225 p255)(includes o225 p290)

(waiting o226)
(includes o226 p10)(includes o226 p158)(includes o226 p181)(includes o226 p203)(includes o226 p267)

(waiting o227)
(includes o227 p33)(includes o227 p58)(includes o227 p71)(includes o227 p83)(includes o227 p89)(includes o227 p106)(includes o227 p152)(includes o227 p184)(includes o227 p289)

(waiting o228)
(includes o228 p90)(includes o228 p254)

(waiting o229)
(includes o229 p11)(includes o229 p41)(includes o229 p57)(includes o229 p68)(includes o229 p77)(includes o229 p107)(includes o229 p126)(includes o229 p215)

(waiting o230)
(includes o230 p37)(includes o230 p78)(includes o230 p143)(includes o230 p175)(includes o230 p182)(includes o230 p275)

(waiting o231)
(includes o231 p83)(includes o231 p114)(includes o231 p155)(includes o231 p183)(includes o231 p290)

(waiting o232)
(includes o232 p36)(includes o232 p62)(includes o232 p63)(includes o232 p67)(includes o232 p133)(includes o232 p245)(includes o232 p250)

(waiting o233)
(includes o233 p1)(includes o233 p12)(includes o233 p16)(includes o233 p93)(includes o233 p113)(includes o233 p137)(includes o233 p142)(includes o233 p195)(includes o233 p210)(includes o233 p220)

(waiting o234)
(includes o234 p106)(includes o234 p130)(includes o234 p140)(includes o234 p143)(includes o234 p146)(includes o234 p155)(includes o234 p193)(includes o234 p244)(includes o234 p249)(includes o234 p263)(includes o234 p270)

(waiting o235)
(includes o235 p6)(includes o235 p115)(includes o235 p125)(includes o235 p138)(includes o235 p154)(includes o235 p164)(includes o235 p209)

(waiting o236)
(includes o236 p20)(includes o236 p43)(includes o236 p125)(includes o236 p142)(includes o236 p148)(includes o236 p192)(includes o236 p270)(includes o236 p274)(includes o236 p278)

(waiting o237)
(includes o237 p10)(includes o237 p59)(includes o237 p96)(includes o237 p136)(includes o237 p175)(includes o237 p179)(includes o237 p240)

(waiting o238)
(includes o238 p32)(includes o238 p74)(includes o238 p88)(includes o238 p99)(includes o238 p105)(includes o238 p113)(includes o238 p152)(includes o238 p190)(includes o238 p191)(includes o238 p208)(includes o238 p212)(includes o238 p223)(includes o238 p229)(includes o238 p259)

(waiting o239)
(includes o239 p43)(includes o239 p49)(includes o239 p93)(includes o239 p192)(includes o239 p204)(includes o239 p231)

(waiting o240)
(includes o240 p148)(includes o240 p176)(includes o240 p283)

(waiting o241)
(includes o241 p38)(includes o241 p112)(includes o241 p188)(includes o241 p240)

(waiting o242)
(includes o242 p35)(includes o242 p44)(includes o242 p74)(includes o242 p110)(includes o242 p120)(includes o242 p159)(includes o242 p209)(includes o242 p262)

(waiting o243)
(includes o243 p30)(includes o243 p47)(includes o243 p96)(includes o243 p109)(includes o243 p140)(includes o243 p144)(includes o243 p151)(includes o243 p182)(includes o243 p205)(includes o243 p259)(includes o243 p271)(includes o243 p284)

(waiting o244)
(includes o244 p22)(includes o244 p108)(includes o244 p213)

(waiting o245)
(includes o245 p11)(includes o245 p12)(includes o245 p14)(includes o245 p123)(includes o245 p186)(includes o245 p196)(includes o245 p215)(includes o245 p237)(includes o245 p267)

(waiting o246)
(includes o246 p19)(includes o246 p102)(includes o246 p135)(includes o246 p200)

(waiting o247)
(includes o247 p17)(includes o247 p28)(includes o247 p34)(includes o247 p57)(includes o247 p82)(includes o247 p139)(includes o247 p176)(includes o247 p178)(includes o247 p228)(includes o247 p233)

(waiting o248)
(includes o248 p2)(includes o248 p59)(includes o248 p88)(includes o248 p102)(includes o248 p116)(includes o248 p172)(includes o248 p176)(includes o248 p206)(includes o248 p217)(includes o248 p255)

(waiting o249)
(includes o249 p46)(includes o249 p63)(includes o249 p89)(includes o249 p92)(includes o249 p135)(includes o249 p174)(includes o249 p178)(includes o249 p191)(includes o249 p218)

(waiting o250)
(includes o250 p164)(includes o250 p171)(includes o250 p194)(includes o250 p249)

(waiting o251)
(includes o251 p5)(includes o251 p17)(includes o251 p19)(includes o251 p21)(includes o251 p22)(includes o251 p50)(includes o251 p101)(includes o251 p130)(includes o251 p202)(includes o251 p205)(includes o251 p213)(includes o251 p257)(includes o251 p260)

(waiting o252)
(includes o252 p2)(includes o252 p9)(includes o252 p22)(includes o252 p25)(includes o252 p52)(includes o252 p76)(includes o252 p126)(includes o252 p165)(includes o252 p185)(includes o252 p236)

(waiting o253)
(includes o253 p68)(includes o253 p99)(includes o253 p121)(includes o253 p139)(includes o253 p211)

(waiting o254)
(includes o254 p3)(includes o254 p51)(includes o254 p63)(includes o254 p71)(includes o254 p85)(includes o254 p90)(includes o254 p93)(includes o254 p126)(includes o254 p186)(includes o254 p268)

(waiting o255)
(includes o255 p1)(includes o255 p14)(includes o255 p33)(includes o255 p41)(includes o255 p58)(includes o255 p60)(includes o255 p61)(includes o255 p94)(includes o255 p119)(includes o255 p166)(includes o255 p169)(includes o255 p190)(includes o255 p238)(includes o255 p249)

(waiting o256)
(includes o256 p52)(includes o256 p87)(includes o256 p105)(includes o256 p119)(includes o256 p125)(includes o256 p236)

(waiting o257)
(includes o257 p233)

(waiting o258)
(includes o258 p27)(includes o258 p51)(includes o258 p234)(includes o258 p235)

(waiting o259)
(includes o259 p10)(includes o259 p24)(includes o259 p65)(includes o259 p105)(includes o259 p108)(includes o259 p194)(includes o259 p267)

(waiting o260)
(includes o260 p32)(includes o260 p69)(includes o260 p96)(includes o260 p101)(includes o260 p152)(includes o260 p201)(includes o260 p271)

(waiting o261)
(includes o261 p12)(includes o261 p20)(includes o261 p98)(includes o261 p112)(includes o261 p167)(includes o261 p281)

(waiting o262)
(includes o262 p32)(includes o262 p144)(includes o262 p203)(includes o262 p230)(includes o262 p235)

(waiting o263)
(includes o263 p19)(includes o263 p46)(includes o263 p139)(includes o263 p261)

(waiting o264)
(includes o264 p35)(includes o264 p180)(includes o264 p206)(includes o264 p248)(includes o264 p249)(includes o264 p263)(includes o264 p265)

(waiting o265)
(includes o265 p70)(includes o265 p80)(includes o265 p192)(includes o265 p240)(includes o265 p242)(includes o265 p249)

(waiting o266)
(includes o266 p73)(includes o266 p118)(includes o266 p136)(includes o266 p142)(includes o266 p175)(includes o266 p209)(includes o266 p236)(includes o266 p248)

(waiting o267)
(includes o267 p68)(includes o267 p91)(includes o267 p191)(includes o267 p225)(includes o267 p279)

(waiting o268)
(includes o268 p45)(includes o268 p71)(includes o268 p173)(includes o268 p186)(includes o268 p214)

(waiting o269)
(includes o269 p167)(includes o269 p188)(includes o269 p213)(includes o269 p243)(includes o269 p250)

(waiting o270)
(includes o270 p90)(includes o270 p151)(includes o270 p214)(includes o270 p236)

(waiting o271)
(includes o271 p1)(includes o271 p9)(includes o271 p39)(includes o271 p140)(includes o271 p141)(includes o271 p153)(includes o271 p156)(includes o271 p241)

(waiting o272)
(includes o272 p132)(includes o272 p203)(includes o272 p205)(includes o272 p215)(includes o272 p229)(includes o272 p279)(includes o272 p287)

(waiting o273)
(includes o273 p5)(includes o273 p14)(includes o273 p106)(includes o273 p114)(includes o273 p140)(includes o273 p142)(includes o273 p147)(includes o273 p149)(includes o273 p150)(includes o273 p152)(includes o273 p171)(includes o273 p228)(includes o273 p244)(includes o273 p253)

(waiting o274)
(includes o274 p14)(includes o274 p29)(includes o274 p41)(includes o274 p84)(includes o274 p136)(includes o274 p146)(includes o274 p165)(includes o274 p193)(includes o274 p194)(includes o274 p245)

(waiting o275)
(includes o275 p173)(includes o275 p215)

(waiting o276)
(includes o276 p13)(includes o276 p20)(includes o276 p102)(includes o276 p265)

(waiting o277)
(includes o277 p79)(includes o277 p154)(includes o277 p158)(includes o277 p186)

(waiting o278)
(includes o278 p11)(includes o278 p15)(includes o278 p30)(includes o278 p70)(includes o278 p79)(includes o278 p139)(includes o278 p167)(includes o278 p197)(includes o278 p257)(includes o278 p272)(includes o278 p280)

(waiting o279)
(includes o279 p26)(includes o279 p173)(includes o279 p189)(includes o279 p193)(includes o279 p199)(includes o279 p200)(includes o279 p205)

(waiting o280)
(includes o280 p150)(includes o280 p221)(includes o280 p240)(includes o280 p266)

(waiting o281)
(includes o281 p26)(includes o281 p29)(includes o281 p51)(includes o281 p77)(includes o281 p80)(includes o281 p109)(includes o281 p120)(includes o281 p166)(includes o281 p187)(includes o281 p270)

(waiting o282)
(includes o282 p137)(includes o282 p156)(includes o282 p284)

(waiting o283)
(includes o283 p8)(includes o283 p20)(includes o283 p42)(includes o283 p92)(includes o283 p176)(includes o283 p178)(includes o283 p208)(includes o283 p235)(includes o283 p284)

(waiting o284)
(includes o284 p96)(includes o284 p168)(includes o284 p195)(includes o284 p212)(includes o284 p215)(includes o284 p243)

(waiting o285)
(includes o285 p16)(includes o285 p48)(includes o285 p89)(includes o285 p118)(includes o285 p162)(includes o285 p230)(includes o285 p258)(includes o285 p275)

(waiting o286)
(includes o286 p3)(includes o286 p30)(includes o286 p121)(includes o286 p135)(includes o286 p150)(includes o286 p196)(includes o286 p202)(includes o286 p216)(includes o286 p242)

(waiting o287)
(includes o287 p1)(includes o287 p60)(includes o287 p73)(includes o287 p96)(includes o287 p97)(includes o287 p103)(includes o287 p110)(includes o287 p128)(includes o287 p198)(includes o287 p201)(includes o287 p279)

(waiting o288)
(includes o288 p2)(includes o288 p25)(includes o288 p74)(includes o288 p113)(includes o288 p276)(includes o288 p283)

(waiting o289)
(includes o289 p42)(includes o289 p54)(includes o289 p76)(includes o289 p77)(includes o289 p95)(includes o289 p109)(includes o289 p114)(includes o289 p159)(includes o289 p214)

(waiting o290)
(includes o290 p14)(includes o290 p61)(includes o290 p73)(includes o290 p97)(includes o290 p147)(includes o290 p216)(includes o290 p248)

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

