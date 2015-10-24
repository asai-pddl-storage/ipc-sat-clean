(define (problem os-sequencedstrips-p290_3)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210 n211 n212 n213 n214 n215 n216 n217 n218 n219 n220 n221 n222 n223 n224 n225 n226 n227 n228 n229 n230 n231 n232 n233 n234 n235 n236 n237 n238 n239 n240 n241 n242 n243 n244 n245 n246 n247 n248 n249 n250 n251 n252 n253 n254 n255 n256 n257 n258 n259 n260 n261 n262 n263 n264 n265 n266 n267 n268 n269 n270 n271 n272 n273 n274 n275 n276 n277 n278 n279 n280 n281 n282 n283 n284 n285 n286 n287 n288 n289 n290  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) (next-count n210 n211) (next-count n211 n212) (next-count n212 n213) (next-count n213 n214) (next-count n214 n215) (next-count n215 n216) (next-count n216 n217) (next-count n217 n218) (next-count n218 n219) (next-count n219 n220) (next-count n220 n221) (next-count n221 n222) (next-count n222 n223) (next-count n223 n224) (next-count n224 n225) (next-count n225 n226) (next-count n226 n227) (next-count n227 n228) (next-count n228 n229) (next-count n229 n230) (next-count n230 n231) (next-count n231 n232) (next-count n232 n233) (next-count n233 n234) (next-count n234 n235) (next-count n235 n236) (next-count n236 n237) (next-count n237 n238) (next-count n238 n239) (next-count n239 n240) (next-count n240 n241) (next-count n241 n242) (next-count n242 n243) (next-count n243 n244) (next-count n244 n245) (next-count n245 n246) (next-count n246 n247) (next-count n247 n248) (next-count n248 n249) (next-count n249 n250) (next-count n250 n251) (next-count n251 n252) (next-count n252 n253) (next-count n253 n254) (next-count n254 n255) (next-count n255 n256) (next-count n256 n257) (next-count n257 n258) (next-count n258 n259) (next-count n259 n260) (next-count n260 n261) (next-count n261 n262) (next-count n262 n263) (next-count n263 n264) (next-count n264 n265) (next-count n265 n266) (next-count n266 n267) (next-count n267 n268) (next-count n268 n269) (next-count n269 n270) (next-count n270 n271) (next-count n271 n272) (next-count n272 n273) (next-count n273 n274) (next-count n274 n275) (next-count n275 n276) (next-count n276 n277) (next-count n277 n278) (next-count n278 n279) (next-count n279 n280) (next-count n280 n281) (next-count n281 n282) (next-count n282 n283) (next-count n283 n284) (next-count n284 n285) (next-count n285 n286) (next-count n286 n287) (next-count n287 n288) (next-count n288 n289) (next-count n289 n290) 
(stacks-avail n0)

(waiting o1)
(includes o1 p52)(includes o1 p73)(includes o1 p84)(includes o1 p94)(includes o1 p101)(includes o1 p142)(includes o1 p160)(includes o1 p185)(includes o1 p265)(includes o1 p274)

(waiting o2)
(includes o2 p103)(includes o2 p111)(includes o2 p169)(includes o2 p191)(includes o2 p217)(includes o2 p283)

(waiting o3)
(includes o3 p11)(includes o3 p50)(includes o3 p104)(includes o3 p136)(includes o3 p174)(includes o3 p175)(includes o3 p214)(includes o3 p286)

(waiting o4)
(includes o4 p41)(includes o4 p64)(includes o4 p105)(includes o4 p143)(includes o4 p220)(includes o4 p237)(includes o4 p288)(includes o4 p289)

(waiting o5)
(includes o5 p68)(includes o5 p69)(includes o5 p72)(includes o5 p111)(includes o5 p152)(includes o5 p191)(includes o5 p210)(includes o5 p241)

(waiting o6)
(includes o6 p15)(includes o6 p23)(includes o6 p81)(includes o6 p105)(includes o6 p155)(includes o6 p212)(includes o6 p242)(includes o6 p257)(includes o6 p267)

(waiting o7)
(includes o7 p32)(includes o7 p54)(includes o7 p61)

(waiting o8)
(includes o8 p20)(includes o8 p61)(includes o8 p92)(includes o8 p130)(includes o8 p198)(includes o8 p230)(includes o8 p242)(includes o8 p258)(includes o8 p288)

(waiting o9)
(includes o9 p18)(includes o9 p70)(includes o9 p86)(includes o9 p98)(includes o9 p101)(includes o9 p146)(includes o9 p155)(includes o9 p170)(includes o9 p215)(includes o9 p234)(includes o9 p249)(includes o9 p269)

(waiting o10)
(includes o10 p13)(includes o10 p17)(includes o10 p40)(includes o10 p93)(includes o10 p154)(includes o10 p165)(includes o10 p177)(includes o10 p184)(includes o10 p233)(includes o10 p256)(includes o10 p275)(includes o10 p279)(includes o10 p288)

(waiting o11)
(includes o11 p39)(includes o11 p42)(includes o11 p186)(includes o11 p193)(includes o11 p205)(includes o11 p218)(includes o11 p254)(includes o11 p255)(includes o11 p260)(includes o11 p272)

(waiting o12)
(includes o12 p18)(includes o12 p79)(includes o12 p126)(includes o12 p144)(includes o12 p213)

(waiting o13)
(includes o13 p37)(includes o13 p141)(includes o13 p173)(includes o13 p196)(includes o13 p247)(includes o13 p280)

(waiting o14)
(includes o14 p58)(includes o14 p64)(includes o14 p102)(includes o14 p108)(includes o14 p120)(includes o14 p130)(includes o14 p264)(includes o14 p269)

(waiting o15)
(includes o15 p70)(includes o15 p140)

(waiting o16)
(includes o16 p6)(includes o16 p63)(includes o16 p104)(includes o16 p142)(includes o16 p225)(includes o16 p260)

(waiting o17)
(includes o17 p9)(includes o17 p134)(includes o17 p156)(includes o17 p205)

(waiting o18)
(includes o18 p36)(includes o18 p53)(includes o18 p112)(includes o18 p115)(includes o18 p129)(includes o18 p158)(includes o18 p162)(includes o18 p169)(includes o18 p179)(includes o18 p274)

(waiting o19)
(includes o19 p150)(includes o19 p162)(includes o19 p237)(includes o19 p287)

(waiting o20)
(includes o20 p30)(includes o20 p71)(includes o20 p97)(includes o20 p220)

(waiting o21)
(includes o21 p88)(includes o21 p95)(includes o21 p151)(includes o21 p171)(includes o21 p177)(includes o21 p202)(includes o21 p249)

(waiting o22)
(includes o22 p2)(includes o22 p63)(includes o22 p67)(includes o22 p73)(includes o22 p100)(includes o22 p111)(includes o22 p179)(includes o22 p190)(includes o22 p193)(includes o22 p202)(includes o22 p219)(includes o22 p254)

(waiting o23)
(includes o23 p29)(includes o23 p49)(includes o23 p123)(includes o23 p153)(includes o23 p224)(includes o23 p283)

(waiting o24)
(includes o24 p1)(includes o24 p8)(includes o24 p47)(includes o24 p176)(includes o24 p211)(includes o24 p276)

(waiting o25)
(includes o25 p43)(includes o25 p48)(includes o25 p71)(includes o25 p77)(includes o25 p107)(includes o25 p139)(includes o25 p195)(includes o25 p253)(includes o25 p254)(includes o25 p272)(includes o25 p277)

(waiting o26)
(includes o26 p14)(includes o26 p39)(includes o26 p57)(includes o26 p73)(includes o26 p122)(includes o26 p129)(includes o26 p189)(includes o26 p190)(includes o26 p191)(includes o26 p264)

(waiting o27)
(includes o27 p55)(includes o27 p119)(includes o27 p140)(includes o27 p141)(includes o27 p175)(includes o27 p225)

(waiting o28)
(includes o28 p73)(includes o28 p88)(includes o28 p255)(includes o28 p290)

(waiting o29)
(includes o29 p102)(includes o29 p128)(includes o29 p168)(includes o29 p187)(includes o29 p236)(includes o29 p283)(includes o29 p287)

(waiting o30)
(includes o30 p71)(includes o30 p73)(includes o30 p138)(includes o30 p148)(includes o30 p202)(includes o30 p218)(includes o30 p226)(includes o30 p261)

(waiting o31)
(includes o31 p55)(includes o31 p63)(includes o31 p119)(includes o31 p176)(includes o31 p199)(includes o31 p200)

(waiting o32)
(includes o32 p19)(includes o32 p33)(includes o32 p54)(includes o32 p65)(includes o32 p101)(includes o32 p170)(includes o32 p230)

(waiting o33)
(includes o33 p28)(includes o33 p70)(includes o33 p79)(includes o33 p103)(includes o33 p156)(includes o33 p173)(includes o33 p183)(includes o33 p198)(includes o33 p215)(includes o33 p236)(includes o33 p241)

(waiting o34)
(includes o34 p76)(includes o34 p167)(includes o34 p183)(includes o34 p229)(includes o34 p239)(includes o34 p258)

(waiting o35)
(includes o35 p74)(includes o35 p104)(includes o35 p203)(includes o35 p216)

(waiting o36)
(includes o36 p10)(includes o36 p53)(includes o36 p60)(includes o36 p72)(includes o36 p85)(includes o36 p103)(includes o36 p104)(includes o36 p145)(includes o36 p162)(includes o36 p178)(includes o36 p203)(includes o36 p206)(includes o36 p217)(includes o36 p221)(includes o36 p241)(includes o36 p260)

(waiting o37)
(includes o37 p10)(includes o37 p194)

(waiting o38)
(includes o38 p78)(includes o38 p82)(includes o38 p152)(includes o38 p179)(includes o38 p245)(includes o38 p277)

(waiting o39)
(includes o39 p9)(includes o39 p104)(includes o39 p109)(includes o39 p126)(includes o39 p241)(includes o39 p252)(includes o39 p280)

(waiting o40)
(includes o40 p1)(includes o40 p38)(includes o40 p45)(includes o40 p80)(includes o40 p159)(includes o40 p165)(includes o40 p181)(includes o40 p200)(includes o40 p240)(includes o40 p270)

(waiting o41)
(includes o41 p18)(includes o41 p27)(includes o41 p86)(includes o41 p101)(includes o41 p152)(includes o41 p189)(includes o41 p190)(includes o41 p288)

(waiting o42)
(includes o42 p9)(includes o42 p44)(includes o42 p111)(includes o42 p133)(includes o42 p135)(includes o42 p149)(includes o42 p208)(includes o42 p271)(includes o42 p276)

(waiting o43)
(includes o43 p99)(includes o43 p102)(includes o43 p156)(includes o43 p278)(includes o43 p284)(includes o43 p287)

(waiting o44)
(includes o44 p15)(includes o44 p68)(includes o44 p195)

(waiting o45)
(includes o45 p70)(includes o45 p91)(includes o45 p101)(includes o45 p129)(includes o45 p196)(includes o45 p197)(includes o45 p227)(includes o45 p229)(includes o45 p234)(includes o45 p274)

(waiting o46)
(includes o46 p114)(includes o46 p152)(includes o46 p163)(includes o46 p182)(includes o46 p186)(includes o46 p273)(includes o46 p287)

(waiting o47)
(includes o47 p1)(includes o47 p6)(includes o47 p45)(includes o47 p73)(includes o47 p193)(includes o47 p254)

(waiting o48)
(includes o48 p48)(includes o48 p112)(includes o48 p151)(includes o48 p172)(includes o48 p175)(includes o48 p190)(includes o48 p206)(includes o48 p221)(includes o48 p242)(includes o48 p257)

(waiting o49)
(includes o49 p35)(includes o49 p38)(includes o49 p95)(includes o49 p173)(includes o49 p194)(includes o49 p198)(includes o49 p199)(includes o49 p248)

(waiting o50)
(includes o50 p2)(includes o50 p188)(includes o50 p189)(includes o50 p230)(includes o50 p231)(includes o50 p261)

(waiting o51)
(includes o51 p7)(includes o51 p78)(includes o51 p85)(includes o51 p98)(includes o51 p107)(includes o51 p115)(includes o51 p136)(includes o51 p150)

(waiting o52)
(includes o52 p24)(includes o52 p57)(includes o52 p142)(includes o52 p216)(includes o52 p239)(includes o52 p280)

(waiting o53)
(includes o53 p63)(includes o53 p101)(includes o53 p108)(includes o53 p115)(includes o53 p133)(includes o53 p155)

(waiting o54)
(includes o54 p2)(includes o54 p38)(includes o54 p43)(includes o54 p50)(includes o54 p58)(includes o54 p73)(includes o54 p85)(includes o54 p116)(includes o54 p181)(includes o54 p214)(includes o54 p239)(includes o54 p260)

(waiting o55)
(includes o55 p35)(includes o55 p113)(includes o55 p290)

(waiting o56)
(includes o56 p5)(includes o56 p20)(includes o56 p37)(includes o56 p68)(includes o56 p91)(includes o56 p135)(includes o56 p204)(includes o56 p227)(includes o56 p238)

(waiting o57)
(includes o57 p33)(includes o57 p51)(includes o57 p102)(includes o57 p134)

(waiting o58)
(includes o58 p155)(includes o58 p164)(includes o58 p165)(includes o58 p198)(includes o58 p243)

(waiting o59)
(includes o59 p29)(includes o59 p266)

(waiting o60)
(includes o60 p63)(includes o60 p74)(includes o60 p82)(includes o60 p83)(includes o60 p96)(includes o60 p126)(includes o60 p127)(includes o60 p204)(includes o60 p228)(includes o60 p280)

(waiting o61)
(includes o61 p43)(includes o61 p47)(includes o61 p52)(includes o61 p152)(includes o61 p189)(includes o61 p217)(includes o61 p239)(includes o61 p246)(includes o61 p272)

(waiting o62)
(includes o62 p22)(includes o62 p171)(includes o62 p200)

(waiting o63)
(includes o63 p21)(includes o63 p120)(includes o63 p128)(includes o63 p139)(includes o63 p146)(includes o63 p192)(includes o63 p211)(includes o63 p218)(includes o63 p227)

(waiting o64)
(includes o64 p34)(includes o64 p115)(includes o64 p148)(includes o64 p225)(includes o64 p279)

(waiting o65)
(includes o65 p2)(includes o65 p52)(includes o65 p133)(includes o65 p165)(includes o65 p177)(includes o65 p186)(includes o65 p233)(includes o65 p239)

(waiting o66)
(includes o66 p20)(includes o66 p59)(includes o66 p60)(includes o66 p65)(includes o66 p88)(includes o66 p212)(includes o66 p215)(includes o66 p218)(includes o66 p219)(includes o66 p226)

(waiting o67)
(includes o67 p21)(includes o67 p46)(includes o67 p52)(includes o67 p80)(includes o67 p130)(includes o67 p165)(includes o67 p171)(includes o67 p279)

(waiting o68)
(includes o68 p5)(includes o68 p47)(includes o68 p62)(includes o68 p79)(includes o68 p154)(includes o68 p226)(includes o68 p255)

(waiting o69)
(includes o69 p2)(includes o69 p28)(includes o69 p31)(includes o69 p81)(includes o69 p195)

(waiting o70)
(includes o70 p88)(includes o70 p94)(includes o70 p249)(includes o70 p261)(includes o70 p288)

(waiting o71)
(includes o71 p25)(includes o71 p42)(includes o71 p43)(includes o71 p44)(includes o71 p79)(includes o71 p104)(includes o71 p178)(includes o71 p190)(includes o71 p194)(includes o71 p265)(includes o71 p272)

(waiting o72)
(includes o72 p33)(includes o72 p79)(includes o72 p87)(includes o72 p143)(includes o72 p150)(includes o72 p156)(includes o72 p181)(includes o72 p271)

(waiting o73)
(includes o73 p35)(includes o73 p55)(includes o73 p112)(includes o73 p113)(includes o73 p115)(includes o73 p126)(includes o73 p133)(includes o73 p196)(includes o73 p241)(includes o73 p246)

(waiting o74)
(includes o74 p43)(includes o74 p54)(includes o74 p86)(includes o74 p115)(includes o74 p190)(includes o74 p192)(includes o74 p198)(includes o74 p216)(includes o74 p221)(includes o74 p257)(includes o74 p267)(includes o74 p276)

(waiting o75)
(includes o75 p52)(includes o75 p209)(includes o75 p231)(includes o75 p238)(includes o75 p290)

(waiting o76)
(includes o76 p16)(includes o76 p74)(includes o76 p178)(includes o76 p182)

(waiting o77)
(includes o77 p42)(includes o77 p60)(includes o77 p100)(includes o77 p162)(includes o77 p189)(includes o77 p193)(includes o77 p197)(includes o77 p213)(includes o77 p247)(includes o77 p285)

(waiting o78)
(includes o78 p2)(includes o78 p32)(includes o78 p44)(includes o78 p56)(includes o78 p146)(includes o78 p151)(includes o78 p157)(includes o78 p277)(includes o78 p289)

(waiting o79)
(includes o79 p45)(includes o79 p96)(includes o79 p136)(includes o79 p157)(includes o79 p218)(includes o79 p250)(includes o79 p259)(includes o79 p286)

(waiting o80)
(includes o80 p31)(includes o80 p33)(includes o80 p149)(includes o80 p159)(includes o80 p191)(includes o80 p257)(includes o80 p269)

(waiting o81)
(includes o81 p26)(includes o81 p34)(includes o81 p60)(includes o81 p72)(includes o81 p94)(includes o81 p106)(includes o81 p127)(includes o81 p136)(includes o81 p218)(includes o81 p272)(includes o81 p275)

(waiting o82)
(includes o82 p34)(includes o82 p85)(includes o82 p199)

(waiting o83)
(includes o83 p8)(includes o83 p35)(includes o83 p76)(includes o83 p123)(includes o83 p138)(includes o83 p147)(includes o83 p175)(includes o83 p233)(includes o83 p253)(includes o83 p260)(includes o83 p273)(includes o83 p280)

(waiting o84)
(includes o84 p30)(includes o84 p40)(includes o84 p105)(includes o84 p232)(includes o84 p283)(includes o84 p287)

(waiting o85)
(includes o85 p75)(includes o85 p132)(includes o85 p285)

(waiting o86)
(includes o86 p96)(includes o86 p110)(includes o86 p269)

(waiting o87)
(includes o87 p55)(includes o87 p93)(includes o87 p176)

(waiting o88)
(includes o88 p2)(includes o88 p40)(includes o88 p89)(includes o88 p159)(includes o88 p161)(includes o88 p188)(includes o88 p216)(includes o88 p222)(includes o88 p261)

(waiting o89)
(includes o89 p99)(includes o89 p153)(includes o89 p165)(includes o89 p270)(includes o89 p281)

(waiting o90)
(includes o90 p47)(includes o90 p79)(includes o90 p84)(includes o90 p123)(includes o90 p147)

(waiting o91)
(includes o91 p60)(includes o91 p162)(includes o91 p167)(includes o91 p184)(includes o91 p195)(includes o91 p206)

(waiting o92)
(includes o92 p14)(includes o92 p45)(includes o92 p47)(includes o92 p134)(includes o92 p236)(includes o92 p254)(includes o92 p290)

(waiting o93)
(includes o93 p82)(includes o93 p136)(includes o93 p180)(includes o93 p197)(includes o93 p202)(includes o93 p226)(includes o93 p252)

(waiting o94)
(includes o94 p43)(includes o94 p54)(includes o94 p80)(includes o94 p136)(includes o94 p167)(includes o94 p219)(includes o94 p229)(includes o94 p236)(includes o94 p239)

(waiting o95)
(includes o95 p24)(includes o95 p74)(includes o95 p155)(includes o95 p177)(includes o95 p185)(includes o95 p193)(includes o95 p206)(includes o95 p230)(includes o95 p232)(includes o95 p250)

(waiting o96)
(includes o96 p28)(includes o96 p102)(includes o96 p126)(includes o96 p195)(includes o96 p208)(includes o96 p246)(includes o96 p280)

(waiting o97)
(includes o97 p8)(includes o97 p56)(includes o97 p105)(includes o97 p197)(includes o97 p230)(includes o97 p235)(includes o97 p276)(includes o97 p283)(includes o97 p287)

(waiting o98)
(includes o98 p47)(includes o98 p103)(includes o98 p148)(includes o98 p164)(includes o98 p179)

(waiting o99)
(includes o99 p18)(includes o99 p27)(includes o99 p28)(includes o99 p95)(includes o99 p170)(includes o99 p180)(includes o99 p284)

(waiting o100)
(includes o100 p4)(includes o100 p63)(includes o100 p87)(includes o100 p131)(includes o100 p211)

(waiting o101)
(includes o101 p11)(includes o101 p13)(includes o101 p19)(includes o101 p32)(includes o101 p40)(includes o101 p46)(includes o101 p57)(includes o101 p135)(includes o101 p207)(includes o101 p240)

(waiting o102)
(includes o102 p21)(includes o102 p63)(includes o102 p70)(includes o102 p107)(includes o102 p108)(includes o102 p109)(includes o102 p122)(includes o102 p222)(includes o102 p254)(includes o102 p277)

(waiting o103)
(includes o103 p54)(includes o103 p70)(includes o103 p87)(includes o103 p97)(includes o103 p109)(includes o103 p140)(includes o103 p228)(includes o103 p248)

(waiting o104)
(includes o104 p25)(includes o104 p97)(includes o104 p132)(includes o104 p143)(includes o104 p199)(includes o104 p246)(includes o104 p264)

(waiting o105)
(includes o105 p89)(includes o105 p96)(includes o105 p130)(includes o105 p151)(includes o105 p229)(includes o105 p270)

(waiting o106)
(includes o106 p14)(includes o106 p97)(includes o106 p140)(includes o106 p168)(includes o106 p174)(includes o106 p176)(includes o106 p201)(includes o106 p222)(includes o106 p226)(includes o106 p248)(includes o106 p266)

(waiting o107)
(includes o107 p16)(includes o107 p24)(includes o107 p59)(includes o107 p100)(includes o107 p112)(includes o107 p127)(includes o107 p168)(includes o107 p174)(includes o107 p213)(includes o107 p221)(includes o107 p266)

(waiting o108)
(includes o108 p5)(includes o108 p45)(includes o108 p103)(includes o108 p105)(includes o108 p133)(includes o108 p136)(includes o108 p185)(includes o108 p201)(includes o108 p253)

(waiting o109)
(includes o109 p36)(includes o109 p94)(includes o109 p109)(includes o109 p217)(includes o109 p222)(includes o109 p230)

(waiting o110)
(includes o110 p6)(includes o110 p92)(includes o110 p120)(includes o110 p245)(includes o110 p261)

(waiting o111)
(includes o111 p34)(includes o111 p58)(includes o111 p137)(includes o111 p141)(includes o111 p197)(includes o111 p207)(includes o111 p235)(includes o111 p239)(includes o111 p261)(includes o111 p281)

(waiting o112)
(includes o112 p3)(includes o112 p46)(includes o112 p257)

(waiting o113)
(includes o113 p9)(includes o113 p25)(includes o113 p82)(includes o113 p169)(includes o113 p173)(includes o113 p262)

(waiting o114)
(includes o114 p19)(includes o114 p67)(includes o114 p80)(includes o114 p96)(includes o114 p112)(includes o114 p146)(includes o114 p153)(includes o114 p173)(includes o114 p228)(includes o114 p229)(includes o114 p233)(includes o114 p287)

(waiting o115)
(includes o115 p40)(includes o115 p66)(includes o115 p69)(includes o115 p103)(includes o115 p151)

(waiting o116)
(includes o116 p80)(includes o116 p130)(includes o116 p141)(includes o116 p157)

(waiting o117)
(includes o117 p13)(includes o117 p36)(includes o117 p44)(includes o117 p112)(includes o117 p144)(includes o117 p223)

(waiting o118)
(includes o118 p77)(includes o118 p108)(includes o118 p128)(includes o118 p233)

(waiting o119)
(includes o119 p49)(includes o119 p174)(includes o119 p245)

(waiting o120)
(includes o120 p260)(includes o120 p268)

(waiting o121)
(includes o121 p12)(includes o121 p19)(includes o121 p65)(includes o121 p130)(includes o121 p157)(includes o121 p210)(includes o121 p281)

(waiting o122)
(includes o122 p75)(includes o122 p183)

(waiting o123)
(includes o123 p92)(includes o123 p123)(includes o123 p160)(includes o123 p171)(includes o123 p256)(includes o123 p265)(includes o123 p286)

(waiting o124)
(includes o124 p30)(includes o124 p90)(includes o124 p159)(includes o124 p191)(includes o124 p222)(includes o124 p234)(includes o124 p251)

(waiting o125)
(includes o125 p6)(includes o125 p17)(includes o125 p200)(includes o125 p216)(includes o125 p236)

(waiting o126)
(includes o126 p19)(includes o126 p45)(includes o126 p80)(includes o126 p166)(includes o126 p245)(includes o126 p277)

(waiting o127)
(includes o127 p3)(includes o127 p75)(includes o127 p81)(includes o127 p130)(includes o127 p144)(includes o127 p167)(includes o127 p213)(includes o127 p223)(includes o127 p227)(includes o127 p263)(includes o127 p279)(includes o127 p282)

(waiting o128)
(includes o128 p58)(includes o128 p65)(includes o128 p69)(includes o128 p204)(includes o128 p267)

(waiting o129)
(includes o129 p2)(includes o129 p25)(includes o129 p42)(includes o129 p110)(includes o129 p122)(includes o129 p160)(includes o129 p182)(includes o129 p267)

(waiting o130)
(includes o130 p14)(includes o130 p54)(includes o130 p113)(includes o130 p159)(includes o130 p233)(includes o130 p246)(includes o130 p254)(includes o130 p255)

(waiting o131)
(includes o131 p72)(includes o131 p132)(includes o131 p189)(includes o131 p202)(includes o131 p228)(includes o131 p236)(includes o131 p260)(includes o131 p262)

(waiting o132)
(includes o132 p38)(includes o132 p155)(includes o132 p192)(includes o132 p197)(includes o132 p229)(includes o132 p289)

(waiting o133)
(includes o133 p42)(includes o133 p124)(includes o133 p190)(includes o133 p207)(includes o133 p245)

(waiting o134)
(includes o134 p42)(includes o134 p100)(includes o134 p104)(includes o134 p138)(includes o134 p140)(includes o134 p157)(includes o134 p173)(includes o134 p176)(includes o134 p194)(includes o134 p202)(includes o134 p262)

(waiting o135)
(includes o135 p3)(includes o135 p44)(includes o135 p49)(includes o135 p123)(includes o135 p189)(includes o135 p243)

(waiting o136)
(includes o136 p9)(includes o136 p24)(includes o136 p25)(includes o136 p98)(includes o136 p110)(includes o136 p132)(includes o136 p156)(includes o136 p175)

(waiting o137)
(includes o137 p10)(includes o137 p45)(includes o137 p200)(includes o137 p205)

(waiting o138)
(includes o138 p90)

(waiting o139)
(includes o139 p39)(includes o139 p118)(includes o139 p146)(includes o139 p152)(includes o139 p159)(includes o139 p190)(includes o139 p209)(includes o139 p277)

(waiting o140)
(includes o140 p8)(includes o140 p74)(includes o140 p89)(includes o140 p122)(includes o140 p132)(includes o140 p146)(includes o140 p160)

(waiting o141)
(includes o141 p22)(includes o141 p53)(includes o141 p200)(includes o141 p205)(includes o141 p263)(includes o141 p277)(includes o141 p282)

(waiting o142)
(includes o142 p69)(includes o142 p113)(includes o142 p119)(includes o142 p165)(includes o142 p169)(includes o142 p186)(includes o142 p229)(includes o142 p239)(includes o142 p261)(includes o142 p285)

(waiting o143)
(includes o143 p17)(includes o143 p18)(includes o143 p32)(includes o143 p76)

(waiting o144)
(includes o144 p39)(includes o144 p108)(includes o144 p125)(includes o144 p150)(includes o144 p160)(includes o144 p168)(includes o144 p207)(includes o144 p222)(includes o144 p236)(includes o144 p262)(includes o144 p265)(includes o144 p268)

(waiting o145)
(includes o145 p69)(includes o145 p74)(includes o145 p82)(includes o145 p85)(includes o145 p116)(includes o145 p132)(includes o145 p208)(includes o145 p210)(includes o145 p247)

(waiting o146)
(includes o146 p5)(includes o146 p41)(includes o146 p79)(includes o146 p81)(includes o146 p103)(includes o146 p186)(includes o146 p206)(includes o146 p229)(includes o146 p256)(includes o146 p284)

(waiting o147)
(includes o147 p32)(includes o147 p93)(includes o147 p127)(includes o147 p132)(includes o147 p185)(includes o147 p198)(includes o147 p219)

(waiting o148)
(includes o148 p109)(includes o148 p144)(includes o148 p285)(includes o148 p287)

(waiting o149)
(includes o149 p63)(includes o149 p65)(includes o149 p144)(includes o149 p180)(includes o149 p214)(includes o149 p241)(includes o149 p242)(includes o149 p257)

(waiting o150)
(includes o150 p88)(includes o150 p165)(includes o150 p218)

(waiting o151)
(includes o151 p1)(includes o151 p66)(includes o151 p90)(includes o151 p120)(includes o151 p208)(includes o151 p219)(includes o151 p220)(includes o151 p245)(includes o151 p285)

(waiting o152)
(includes o152 p24)(includes o152 p51)(includes o152 p202)(includes o152 p256)(includes o152 p259)

(waiting o153)
(includes o153 p21)(includes o153 p39)(includes o153 p127)(includes o153 p241)

(waiting o154)
(includes o154 p144)(includes o154 p159)(includes o154 p230)(includes o154 p235)(includes o154 p288)

(waiting o155)
(includes o155 p134)(includes o155 p160)(includes o155 p164)(includes o155 p253)

(waiting o156)
(includes o156 p37)(includes o156 p43)(includes o156 p119)(includes o156 p124)

(waiting o157)
(includes o157 p29)(includes o157 p31)(includes o157 p66)(includes o157 p114)(includes o157 p119)(includes o157 p127)(includes o157 p204)

(waiting o158)
(includes o158 p4)(includes o158 p9)(includes o158 p30)(includes o158 p31)(includes o158 p36)(includes o158 p69)(includes o158 p76)(includes o158 p110)(includes o158 p135)(includes o158 p198)(includes o158 p260)

(waiting o159)
(includes o159 p4)(includes o159 p38)(includes o159 p73)(includes o159 p123)

(waiting o160)
(includes o160 p37)(includes o160 p108)(includes o160 p130)(includes o160 p198)(includes o160 p202)(includes o160 p244)(includes o160 p251)

(waiting o161)
(includes o161 p34)(includes o161 p82)(includes o161 p180)(includes o161 p191)(includes o161 p220)(includes o161 p221)(includes o161 p226)(includes o161 p228)

(waiting o162)
(includes o162 p13)(includes o162 p55)(includes o162 p58)(includes o162 p63)(includes o162 p75)(includes o162 p128)(includes o162 p137)(includes o162 p171)(includes o162 p213)(includes o162 p264)

(waiting o163)
(includes o163 p108)(includes o163 p254)(includes o163 p262)

(waiting o164)
(includes o164 p10)(includes o164 p23)(includes o164 p25)(includes o164 p45)(includes o164 p73)(includes o164 p93)(includes o164 p104)(includes o164 p118)(includes o164 p122)(includes o164 p198)(includes o164 p210)(includes o164 p214)(includes o164 p270)(includes o164 p274)(includes o164 p281)

(waiting o165)
(includes o165 p3)(includes o165 p36)(includes o165 p78)(includes o165 p88)(includes o165 p151)(includes o165 p165)(includes o165 p255)(includes o165 p264)

(waiting o166)
(includes o166 p21)(includes o166 p80)(includes o166 p159)(includes o166 p174)(includes o166 p229)(includes o166 p290)

(waiting o167)
(includes o167 p64)(includes o167 p70)(includes o167 p85)(includes o167 p90)(includes o167 p113)(includes o167 p114)(includes o167 p129)(includes o167 p232)(includes o167 p242)(includes o167 p278)

(waiting o168)
(includes o168 p12)(includes o168 p16)(includes o168 p20)(includes o168 p105)(includes o168 p195)(includes o168 p269)(includes o168 p285)

(waiting o169)
(includes o169 p23)(includes o169 p36)(includes o169 p83)(includes o169 p85)(includes o169 p87)(includes o169 p108)(includes o169 p147)(includes o169 p152)(includes o169 p217)(includes o169 p254)

(waiting o170)
(includes o170 p35)(includes o170 p110)(includes o170 p157)(includes o170 p162)(includes o170 p197)(includes o170 p204)

(waiting o171)
(includes o171 p236)(includes o171 p253)

(waiting o172)
(includes o172 p13)(includes o172 p17)(includes o172 p131)(includes o172 p207)(includes o172 p221)(includes o172 p259)(includes o172 p277)

(waiting o173)
(includes o173 p3)(includes o173 p6)(includes o173 p84)(includes o173 p144)(includes o173 p200)(includes o173 p278)

(waiting o174)
(includes o174 p61)(includes o174 p98)(includes o174 p135)(includes o174 p145)(includes o174 p146)

(waiting o175)
(includes o175 p13)(includes o175 p14)(includes o175 p21)(includes o175 p101)(includes o175 p108)(includes o175 p181)(includes o175 p219)(includes o175 p227)(includes o175 p229)

(waiting o176)
(includes o176 p16)(includes o176 p59)(includes o176 p74)(includes o176 p97)(includes o176 p118)(includes o176 p138)(includes o176 p143)(includes o176 p179)(includes o176 p221)(includes o176 p241)(includes o176 p249)(includes o176 p278)(includes o176 p280)

(waiting o177)
(includes o177 p108)(includes o177 p112)(includes o177 p145)(includes o177 p174)(includes o177 p189)(includes o177 p254)

(waiting o178)
(includes o178 p16)(includes o178 p49)(includes o178 p68)(includes o178 p154)(includes o178 p199)(includes o178 p218)(includes o178 p241)(includes o178 p256)

(waiting o179)
(includes o179 p37)(includes o179 p82)(includes o179 p213)(includes o179 p243)(includes o179 p256)(includes o179 p289)

(waiting o180)
(includes o180 p31)(includes o180 p60)(includes o180 p67)(includes o180 p69)(includes o180 p165)(includes o180 p232)(includes o180 p234)(includes o180 p256)

(waiting o181)
(includes o181 p19)(includes o181 p56)(includes o181 p71)(includes o181 p131)(includes o181 p135)(includes o181 p215)(includes o181 p261)(includes o181 p267)(includes o181 p286)

(waiting o182)
(includes o182 p31)(includes o182 p59)(includes o182 p80)(includes o182 p82)(includes o182 p118)(includes o182 p153)(includes o182 p157)(includes o182 p176)(includes o182 p247)(includes o182 p248)

(waiting o183)
(includes o183 p11)(includes o183 p23)(includes o183 p91)(includes o183 p125)(includes o183 p127)(includes o183 p135)(includes o183 p156)(includes o183 p159)(includes o183 p226)(includes o183 p227)(includes o183 p251)(includes o183 p257)

(waiting o184)
(includes o184 p167)(includes o184 p188)

(waiting o185)
(includes o185 p53)(includes o185 p107)(includes o185 p141)(includes o185 p152)(includes o185 p180)(includes o185 p188)(includes o185 p210)(includes o185 p217)

(waiting o186)
(includes o186 p136)(includes o186 p150)(includes o186 p234)(includes o186 p260)

(waiting o187)
(includes o187 p49)(includes o187 p55)(includes o187 p90)(includes o187 p132)(includes o187 p264)(includes o187 p271)

(waiting o188)
(includes o188 p17)(includes o188 p18)(includes o188 p41)(includes o188 p61)(includes o188 p107)(includes o188 p138)(includes o188 p173)(includes o188 p187)(includes o188 p194)(includes o188 p267)

(waiting o189)
(includes o189 p9)(includes o189 p37)(includes o189 p61)(includes o189 p116)(includes o189 p138)(includes o189 p148)(includes o189 p234)(includes o189 p276)(includes o189 p283)

(waiting o190)
(includes o190 p8)(includes o190 p99)(includes o190 p135)(includes o190 p139)(includes o190 p154)(includes o190 p161)(includes o190 p204)(includes o190 p234)

(waiting o191)
(includes o191 p57)(includes o191 p63)(includes o191 p100)(includes o191 p194)(includes o191 p200)(includes o191 p210)(includes o191 p212)(includes o191 p222)

(waiting o192)
(includes o192 p8)(includes o192 p26)(includes o192 p64)(includes o192 p99)(includes o192 p123)(includes o192 p132)(includes o192 p147)(includes o192 p237)

(waiting o193)
(includes o193 p4)(includes o193 p6)(includes o193 p39)(includes o193 p58)(includes o193 p83)(includes o193 p181)(includes o193 p188)(includes o193 p205)(includes o193 p246)(includes o193 p257)(includes o193 p262)

(waiting o194)
(includes o194 p53)(includes o194 p76)(includes o194 p95)(includes o194 p109)(includes o194 p112)(includes o194 p133)(includes o194 p147)(includes o194 p163)(includes o194 p212)

(waiting o195)
(includes o195 p16)(includes o195 p22)(includes o195 p111)(includes o195 p122)(includes o195 p218)(includes o195 p230)(includes o195 p239)(includes o195 p246)(includes o195 p249)(includes o195 p260)(includes o195 p261)(includes o195 p288)

(waiting o196)
(includes o196 p50)(includes o196 p54)(includes o196 p82)(includes o196 p129)(includes o196 p134)(includes o196 p151)(includes o196 p188)(includes o196 p215)(includes o196 p250)(includes o196 p270)(includes o196 p276)

(waiting o197)
(includes o197 p5)(includes o197 p50)(includes o197 p185)(includes o197 p194)(includes o197 p258)

(waiting o198)
(includes o198 p70)(includes o198 p89)(includes o198 p153)(includes o198 p173)(includes o198 p175)(includes o198 p216)(includes o198 p241)(includes o198 p263)(includes o198 p274)(includes o198 p283)

(waiting o199)
(includes o199 p2)(includes o199 p140)(includes o199 p145)(includes o199 p239)(includes o199 p264)(includes o199 p279)

(waiting o200)
(includes o200 p79)(includes o200 p85)(includes o200 p234)(includes o200 p241)(includes o200 p267)(includes o200 p275)

(waiting o201)
(includes o201 p21)(includes o201 p45)(includes o201 p195)(includes o201 p202)(includes o201 p277)

(waiting o202)
(includes o202 p25)(includes o202 p26)(includes o202 p68)(includes o202 p111)(includes o202 p149)(includes o202 p228)(includes o202 p232)

(waiting o203)
(includes o203 p113)(includes o203 p129)(includes o203 p139)(includes o203 p172)(includes o203 p219)(includes o203 p244)

(waiting o204)
(includes o204 p20)(includes o204 p34)(includes o204 p222)

(waiting o205)
(includes o205 p62)(includes o205 p67)(includes o205 p105)(includes o205 p143)(includes o205 p178)(includes o205 p193)(includes o205 p285)

(waiting o206)
(includes o206 p21)(includes o206 p23)(includes o206 p97)(includes o206 p169)(includes o206 p204)(includes o206 p235)(includes o206 p240)(includes o206 p269)

(waiting o207)
(includes o207 p50)(includes o207 p81)(includes o207 p82)(includes o207 p96)(includes o207 p137)(includes o207 p156)(includes o207 p172)(includes o207 p261)

(waiting o208)
(includes o208 p20)(includes o208 p103)(includes o208 p174)(includes o208 p244)

(waiting o209)
(includes o209 p64)(includes o209 p84)(includes o209 p113)(includes o209 p132)(includes o209 p275)

(waiting o210)
(includes o210 p5)(includes o210 p7)(includes o210 p149)(includes o210 p192)(includes o210 p215)

(waiting o211)
(includes o211 p1)(includes o211 p66)(includes o211 p140)(includes o211 p143)(includes o211 p225)

(waiting o212)
(includes o212 p18)(includes o212 p29)(includes o212 p75)(includes o212 p106)(includes o212 p111)(includes o212 p234)

(waiting o213)
(includes o213 p54)(includes o213 p134)(includes o213 p144)

(waiting o214)
(includes o214 p69)(includes o214 p136)(includes o214 p152)(includes o214 p190)(includes o214 p223)(includes o214 p231)(includes o214 p242)

(waiting o215)
(includes o215 p23)(includes o215 p67)(includes o215 p79)(includes o215 p82)(includes o215 p86)(includes o215 p214)(includes o215 p262)(includes o215 p287)

(waiting o216)
(includes o216 p3)(includes o216 p7)(includes o216 p49)(includes o216 p75)(includes o216 p81)(includes o216 p274)

(waiting o217)
(includes o217 p11)(includes o217 p30)(includes o217 p41)(includes o217 p76)(includes o217 p133)(includes o217 p138)(includes o217 p256)(includes o217 p280)

(waiting o218)
(includes o218 p35)(includes o218 p63)(includes o218 p144)(includes o218 p202)(includes o218 p210)(includes o218 p272)

(waiting o219)
(includes o219 p9)(includes o219 p42)(includes o219 p85)(includes o219 p97)(includes o219 p186)(includes o219 p206)(includes o219 p244)

(waiting o220)
(includes o220 p169)(includes o220 p179)(includes o220 p206)(includes o220 p216)(includes o220 p221)(includes o220 p272)(includes o220 p282)

(waiting o221)
(includes o221 p10)(includes o221 p24)(includes o221 p37)(includes o221 p50)(includes o221 p54)(includes o221 p128)(includes o221 p186)(includes o221 p197)(includes o221 p227)(includes o221 p281)

(waiting o222)
(includes o222 p31)(includes o222 p98)(includes o222 p104)(includes o222 p138)(includes o222 p195)(includes o222 p204)

(waiting o223)
(includes o223 p57)(includes o223 p92)(includes o223 p101)(includes o223 p130)(includes o223 p166)(includes o223 p177)(includes o223 p235)(includes o223 p265)

(waiting o224)
(includes o224 p1)(includes o224 p13)(includes o224 p33)(includes o224 p58)(includes o224 p134)(includes o224 p142)(includes o224 p149)(includes o224 p150)(includes o224 p170)(includes o224 p189)(includes o224 p195)(includes o224 p233)(includes o224 p235)(includes o224 p274)

(waiting o225)
(includes o225 p27)(includes o225 p34)(includes o225 p51)(includes o225 p107)(includes o225 p211)(includes o225 p266)(includes o225 p276)(includes o225 p281)

(waiting o226)
(includes o226 p5)(includes o226 p69)(includes o226 p89)(includes o226 p245)

(waiting o227)
(includes o227 p89)(includes o227 p205)(includes o227 p268)(includes o227 p281)

(waiting o228)
(includes o228 p17)(includes o228 p19)(includes o228 p20)(includes o228 p95)(includes o228 p129)(includes o228 p142)(includes o228 p202)

(waiting o229)
(includes o229 p48)(includes o229 p74)(includes o229 p213)(includes o229 p243)

(waiting o230)
(includes o230 p26)(includes o230 p131)

(waiting o231)
(includes o231 p11)(includes o231 p77)(includes o231 p184)(includes o231 p219)(includes o231 p243)

(waiting o232)
(includes o232 p194)(includes o232 p227)(includes o232 p247)(includes o232 p256)(includes o232 p281)

(waiting o233)
(includes o233 p11)(includes o233 p152)(includes o233 p169)(includes o233 p191)(includes o233 p196)(includes o233 p198)(includes o233 p199)(includes o233 p226)(includes o233 p281)

(waiting o234)
(includes o234 p10)(includes o234 p29)(includes o234 p53)(includes o234 p84)(includes o234 p213)(includes o234 p219)(includes o234 p228)(includes o234 p254)(includes o234 p260)

(waiting o235)
(includes o235 p9)(includes o235 p28)(includes o235 p84)(includes o235 p135)(includes o235 p166)(includes o235 p274)(includes o235 p276)(includes o235 p279)

(waiting o236)
(includes o236 p13)(includes o236 p25)(includes o236 p92)(includes o236 p141)(includes o236 p162)(includes o236 p170)(includes o236 p211)(includes o236 p216)

(waiting o237)
(includes o237 p42)(includes o237 p69)(includes o237 p111)(includes o237 p225)(includes o237 p253)

(waiting o238)
(includes o238 p121)(includes o238 p127)(includes o238 p138)(includes o238 p190)(includes o238 p242)(includes o238 p278)

(waiting o239)
(includes o239 p46)(includes o239 p87)(includes o239 p94)(includes o239 p109)(includes o239 p205)(includes o239 p278)(includes o239 p286)

(waiting o240)
(includes o240 p156)(includes o240 p169)

(waiting o241)
(includes o241 p35)(includes o241 p184)(includes o241 p256)(includes o241 p270)(includes o241 p272)(includes o241 p279)

(waiting o242)
(includes o242 p30)(includes o242 p51)(includes o242 p62)(includes o242 p85)(includes o242 p107)(includes o242 p130)(includes o242 p132)(includes o242 p262)(includes o242 p272)

(waiting o243)
(includes o243 p11)(includes o243 p20)(includes o243 p21)(includes o243 p50)(includes o243 p80)(includes o243 p136)(includes o243 p172)(includes o243 p175)(includes o243 p178)(includes o243 p180)

(waiting o244)
(includes o244 p56)(includes o244 p232)(includes o244 p259)

(waiting o245)
(includes o245 p32)(includes o245 p62)(includes o245 p72)(includes o245 p106)(includes o245 p131)(includes o245 p132)(includes o245 p183)(includes o245 p188)(includes o245 p230)(includes o245 p254)(includes o245 p290)

(waiting o246)
(includes o246 p10)(includes o246 p80)(includes o246 p117)(includes o246 p123)(includes o246 p163)(includes o246 p165)(includes o246 p173)(includes o246 p193)

(waiting o247)
(includes o247 p53)(includes o247 p121)(includes o247 p143)(includes o247 p159)(includes o247 p201)(includes o247 p223)(includes o247 p247)(includes o247 p274)

(waiting o248)
(includes o248 p96)(includes o248 p131)(includes o248 p165)(includes o248 p239)(includes o248 p246)(includes o248 p249)(includes o248 p277)

(waiting o249)
(includes o249 p17)(includes o249 p90)(includes o249 p120)(includes o249 p169)(includes o249 p223)

(waiting o250)
(includes o250 p53)(includes o250 p119)(includes o250 p135)(includes o250 p158)(includes o250 p181)(includes o250 p197)(includes o250 p286)

(waiting o251)
(includes o251 p48)(includes o251 p49)(includes o251 p89)(includes o251 p101)(includes o251 p114)(includes o251 p150)(includes o251 p193)(includes o251 p194)(includes o251 p224)(includes o251 p242)(includes o251 p254)(includes o251 p271)

(waiting o252)
(includes o252 p42)(includes o252 p68)(includes o252 p90)(includes o252 p115)(includes o252 p140)(includes o252 p147)(includes o252 p246)(includes o252 p274)

(waiting o253)
(includes o253 p40)(includes o253 p93)(includes o253 p130)(includes o253 p251)

(waiting o254)
(includes o254 p77)(includes o254 p85)(includes o254 p190)(includes o254 p198)(includes o254 p240)(includes o254 p242)

(waiting o255)
(includes o255 p115)(includes o255 p156)(includes o255 p207)

(waiting o256)
(includes o256 p79)(includes o256 p92)(includes o256 p179)

(waiting o257)
(includes o257 p4)(includes o257 p6)(includes o257 p100)(includes o257 p131)(includes o257 p150)(includes o257 p165)(includes o257 p178)(includes o257 p215)(includes o257 p228)

(waiting o258)
(includes o258 p5)(includes o258 p81)(includes o258 p97)(includes o258 p119)(includes o258 p138)(includes o258 p140)

(waiting o259)
(includes o259 p28)(includes o259 p64)(includes o259 p128)(includes o259 p147)

(waiting o260)
(includes o260 p15)(includes o260 p28)(includes o260 p44)(includes o260 p260)

(waiting o261)
(includes o261 p30)(includes o261 p64)(includes o261 p76)(includes o261 p159)(includes o261 p175)(includes o261 p264)

(waiting o262)
(includes o262 p188)(includes o262 p193)(includes o262 p212)(includes o262 p265)

(waiting o263)
(includes o263 p66)(includes o263 p105)(includes o263 p116)(includes o263 p120)(includes o263 p156)(includes o263 p269)(includes o263 p280)(includes o263 p283)(includes o263 p288)

(waiting o264)
(includes o264 p44)(includes o264 p72)(includes o264 p84)(includes o264 p86)(includes o264 p93)(includes o264 p135)(includes o264 p146)(includes o264 p157)(includes o264 p166)(includes o264 p255)(includes o264 p266)(includes o264 p270)

(waiting o265)
(includes o265 p4)(includes o265 p43)(includes o265 p88)(includes o265 p110)(includes o265 p215)(includes o265 p252)(includes o265 p278)

(waiting o266)
(includes o266 p32)(includes o266 p125)(includes o266 p174)(includes o266 p278)

(waiting o267)
(includes o267 p6)(includes o267 p25)(includes o267 p189)(includes o267 p232)(includes o267 p241)(includes o267 p253)(includes o267 p274)

(waiting o268)
(includes o268 p26)(includes o268 p87)(includes o268 p211)

(waiting o269)
(includes o269 p2)(includes o269 p110)(includes o269 p119)(includes o269 p163)

(waiting o270)
(includes o270 p26)(includes o270 p42)(includes o270 p50)(includes o270 p104)(includes o270 p180)(includes o270 p205)(includes o270 p210)(includes o270 p252)(includes o270 p266)

(waiting o271)
(includes o271 p83)(includes o271 p91)(includes o271 p119)

(waiting o272)
(includes o272 p100)(includes o272 p142)

(waiting o273)
(includes o273 p23)(includes o273 p27)(includes o273 p43)(includes o273 p47)(includes o273 p70)(includes o273 p91)(includes o273 p190)(includes o273 p227)

(waiting o274)
(includes o274 p1)(includes o274 p24)(includes o274 p237)

(waiting o275)
(includes o275 p38)(includes o275 p78)(includes o275 p101)(includes o275 p118)(includes o275 p127)(includes o275 p155)(includes o275 p174)(includes o275 p191)(includes o275 p218)(includes o275 p243)

(waiting o276)
(includes o276 p10)(includes o276 p15)(includes o276 p81)(includes o276 p114)(includes o276 p178)(includes o276 p246)(includes o276 p258)

(waiting o277)
(includes o277 p30)(includes o277 p85)(includes o277 p212)(includes o277 p274)(includes o277 p286)

(waiting o278)
(includes o278 p25)(includes o278 p39)(includes o278 p125)(includes o278 p203)(includes o278 p248)(includes o278 p253)

(waiting o279)
(includes o279 p4)(includes o279 p18)(includes o279 p135)(includes o279 p153)(includes o279 p196)(includes o279 p273)

(waiting o280)
(includes o280 p19)(includes o280 p160)(includes o280 p184)(includes o280 p213)(includes o280 p277)(includes o280 p289)

(waiting o281)
(includes o281 p125)(includes o281 p148)

(waiting o282)
(includes o282 p13)(includes o282 p118)(includes o282 p140)(includes o282 p149)(includes o282 p178)(includes o282 p220)(includes o282 p255)(includes o282 p271)(includes o282 p287)

(waiting o283)
(includes o283 p84)(includes o283 p129)(includes o283 p247)(includes o283 p252)(includes o283 p262)

(waiting o284)
(includes o284 p248)

(waiting o285)
(includes o285 p79)(includes o285 p174)(includes o285 p180)(includes o285 p252)(includes o285 p287)

(waiting o286)
(includes o286 p26)(includes o286 p66)(includes o286 p105)(includes o286 p143)(includes o286 p255)

(waiting o287)
(includes o287 p68)(includes o287 p70)(includes o287 p96)(includes o287 p101)(includes o287 p111)(includes o287 p113)(includes o287 p114)(includes o287 p209)(includes o287 p213)(includes o287 p237)(includes o287 p255)(includes o287 p267)

(waiting o288)
(includes o288 p70)(includes o288 p97)(includes o288 p204)(includes o288 p209)(includes o288 p244)(includes o288 p245)(includes o288 p256)

(waiting o289)
(includes o289 p59)(includes o289 p107)(includes o289 p111)(includes o289 p118)(includes o289 p158)(includes o289 p203)(includes o289 p206)(includes o289 p252)(includes o289 p268)(includes o289 p288)

(waiting o290)
(includes o290 p8)(includes o290 p37)(includes o290 p75)(includes o290 p83)(includes o290 p91)(includes o290 p98)(includes o290 p111)(includes o290 p198)(includes o290 p224)

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

