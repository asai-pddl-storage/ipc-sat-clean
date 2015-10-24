(define (problem os-sequencedstrips-p250_1)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210 n211 n212 n213 n214 n215 n216 n217 n218 n219 n220 n221 n222 n223 n224 n225 n226 n227 n228 n229 n230 n231 n232 n233 n234 n235 n236 n237 n238 n239 n240 n241 n242 n243 n244 n245 n246 n247 n248 n249 n250  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) (next-count n210 n211) (next-count n211 n212) (next-count n212 n213) (next-count n213 n214) (next-count n214 n215) (next-count n215 n216) (next-count n216 n217) (next-count n217 n218) (next-count n218 n219) (next-count n219 n220) (next-count n220 n221) (next-count n221 n222) (next-count n222 n223) (next-count n223 n224) (next-count n224 n225) (next-count n225 n226) (next-count n226 n227) (next-count n227 n228) (next-count n228 n229) (next-count n229 n230) (next-count n230 n231) (next-count n231 n232) (next-count n232 n233) (next-count n233 n234) (next-count n234 n235) (next-count n235 n236) (next-count n236 n237) (next-count n237 n238) (next-count n238 n239) (next-count n239 n240) (next-count n240 n241) (next-count n241 n242) (next-count n242 n243) (next-count n243 n244) (next-count n244 n245) (next-count n245 n246) (next-count n246 n247) (next-count n247 n248) (next-count n248 n249) (next-count n249 n250) 
(stacks-avail n0)

(waiting o1)
(includes o1 p20)(includes o1 p48)(includes o1 p70)(includes o1 p107)(includes o1 p156)(includes o1 p184)(includes o1 p189)(includes o1 p226)

(waiting o2)
(includes o2 p2)(includes o2 p41)(includes o2 p121)(includes o2 p129)(includes o2 p165)(includes o2 p169)(includes o2 p171)(includes o2 p178)(includes o2 p186)(includes o2 p187)(includes o2 p191)(includes o2 p221)

(waiting o3)
(includes o3 p141)(includes o3 p142)(includes o3 p152)(includes o3 p210)(includes o3 p224)(includes o3 p234)(includes o3 p235)

(waiting o4)
(includes o4 p59)(includes o4 p82)(includes o4 p114)(includes o4 p118)

(waiting o5)
(includes o5 p12)(includes o5 p94)(includes o5 p99)(includes o5 p165)(includes o5 p202)(includes o5 p212)(includes o5 p227)

(waiting o6)
(includes o6 p48)(includes o6 p81)(includes o6 p95)(includes o6 p111)(includes o6 p118)(includes o6 p244)

(waiting o7)
(includes o7 p96)(includes o7 p121)(includes o7 p129)(includes o7 p169)(includes o7 p170)(includes o7 p217)(includes o7 p245)

(waiting o8)
(includes o8 p30)(includes o8 p62)(includes o8 p63)(includes o8 p131)(includes o8 p163)(includes o8 p250)

(waiting o9)
(includes o9 p51)(includes o9 p103)(includes o9 p152)(includes o9 p157)(includes o9 p205)(includes o9 p236)

(waiting o10)
(includes o10 p37)(includes o10 p39)(includes o10 p104)(includes o10 p110)(includes o10 p139)(includes o10 p165)(includes o10 p192)(includes o10 p217)

(waiting o11)
(includes o11 p103)(includes o11 p119)(includes o11 p133)

(waiting o12)
(includes o12 p125)(includes o12 p188)(includes o12 p198)(includes o12 p238)

(waiting o13)
(includes o13 p82)(includes o13 p89)(includes o13 p124)(includes o13 p156)(includes o13 p196)

(waiting o14)
(includes o14 p87)(includes o14 p104)(includes o14 p115)(includes o14 p131)

(waiting o15)
(includes o15 p11)(includes o15 p12)(includes o15 p46)(includes o15 p47)(includes o15 p77)(includes o15 p86)(includes o15 p174)

(waiting o16)
(includes o16 p49)(includes o16 p74)(includes o16 p156)(includes o16 p169)(includes o16 p176)(includes o16 p178)(includes o16 p214)(includes o16 p221)

(waiting o17)
(includes o17 p6)(includes o17 p23)(includes o17 p33)(includes o17 p111)(includes o17 p119)(includes o17 p135)(includes o17 p180)(includes o17 p209)(includes o17 p221)

(waiting o18)
(includes o18 p5)(includes o18 p23)(includes o18 p39)(includes o18 p106)(includes o18 p137)(includes o18 p191)

(waiting o19)
(includes o19 p26)(includes o19 p56)(includes o19 p76)(includes o19 p82)(includes o19 p128)(includes o19 p183)(includes o19 p196)

(waiting o20)
(includes o20 p4)(includes o20 p139)(includes o20 p224)

(waiting o21)
(includes o21 p32)(includes o21 p95)(includes o21 p188)

(waiting o22)
(includes o22 p80)(includes o22 p84)(includes o22 p144)(includes o22 p174)

(waiting o23)
(includes o23 p54)(includes o23 p126)(includes o23 p136)(includes o23 p137)

(waiting o24)
(includes o24 p70)(includes o24 p138)(includes o24 p151)(includes o24 p184)

(waiting o25)
(includes o25 p17)(includes o25 p51)(includes o25 p72)(includes o25 p130)(includes o25 p164)(includes o25 p166)(includes o25 p176)

(waiting o26)
(includes o26 p3)(includes o26 p36)(includes o26 p62)(includes o26 p71)(includes o26 p73)(includes o26 p83)(includes o26 p109)(includes o26 p162)(includes o26 p211)(includes o26 p236)

(waiting o27)
(includes o27 p52)

(waiting o28)
(includes o28 p24)(includes o28 p64)(includes o28 p140)(includes o28 p177)(includes o28 p188)

(waiting o29)
(includes o29 p10)(includes o29 p22)(includes o29 p43)(includes o29 p64)(includes o29 p123)(includes o29 p129)

(waiting o30)
(includes o30 p77)(includes o30 p94)(includes o30 p110)(includes o30 p140)(includes o30 p142)(includes o30 p148)(includes o30 p222)

(waiting o31)
(includes o31 p14)(includes o31 p141)(includes o31 p198)(includes o31 p241)

(waiting o32)
(includes o32 p221)(includes o32 p228)

(waiting o33)
(includes o33 p27)(includes o33 p32)(includes o33 p68)(includes o33 p82)(includes o33 p89)(includes o33 p212)(includes o33 p216)

(waiting o34)
(includes o34 p150)

(waiting o35)
(includes o35 p30)(includes o35 p65)(includes o35 p109)(includes o35 p116)(includes o35 p126)(includes o35 p178)(includes o35 p212)(includes o35 p219)(includes o35 p225)(includes o35 p229)

(waiting o36)
(includes o36 p20)(includes o36 p32)(includes o36 p35)(includes o36 p60)(includes o36 p77)(includes o36 p148)(includes o36 p231)(includes o36 p235)

(waiting o37)
(includes o37 p30)(includes o37 p35)(includes o37 p38)(includes o37 p43)(includes o37 p173)(includes o37 p213)(includes o37 p239)

(waiting o38)
(includes o38 p4)(includes o38 p16)(includes o38 p17)(includes o38 p36)(includes o38 p123)(includes o38 p212)

(waiting o39)
(includes o39 p26)(includes o39 p84)(includes o39 p90)(includes o39 p178)(includes o39 p187)(includes o39 p234)(includes o39 p241)

(waiting o40)
(includes o40 p5)(includes o40 p16)(includes o40 p54)(includes o40 p77)(includes o40 p155)(includes o40 p162)(includes o40 p179)(includes o40 p191)(includes o40 p238)(includes o40 p239)

(waiting o41)
(includes o41 p65)(includes o41 p126)(includes o41 p204)(includes o41 p209)(includes o41 p239)

(waiting o42)
(includes o42 p26)(includes o42 p76)(includes o42 p103)(includes o42 p122)(includes o42 p149)(includes o42 p163)(includes o42 p212)

(waiting o43)
(includes o43 p6)(includes o43 p30)(includes o43 p33)(includes o43 p43)(includes o43 p136)

(waiting o44)
(includes o44 p50)(includes o44 p52)(includes o44 p54)(includes o44 p141)(includes o44 p206)(includes o44 p213)

(waiting o45)
(includes o45 p35)(includes o45 p86)(includes o45 p105)(includes o45 p114)(includes o45 p122)

(waiting o46)
(includes o46 p30)(includes o46 p58)(includes o46 p102)(includes o46 p107)(includes o46 p111)(includes o46 p138)(includes o46 p142)(includes o46 p172)(includes o46 p181)(includes o46 p201)(includes o46 p228)(includes o46 p242)

(waiting o47)
(includes o47 p73)(includes o47 p90)(includes o47 p169)(includes o47 p211)(includes o47 p212)(includes o47 p230)(includes o47 p247)

(waiting o48)
(includes o48 p83)(includes o48 p116)(includes o48 p121)(includes o48 p176)(includes o48 p226)(includes o48 p242)(includes o48 p245)

(waiting o49)
(includes o49 p42)(includes o49 p57)(includes o49 p158)(includes o49 p177)(includes o49 p182)

(waiting o50)
(includes o50 p33)(includes o50 p127)(includes o50 p136)(includes o50 p177)(includes o50 p191)(includes o50 p196)

(waiting o51)
(includes o51 p3)(includes o51 p4)(includes o51 p38)(includes o51 p40)(includes o51 p51)(includes o51 p75)(includes o51 p128)(includes o51 p173)(includes o51 p194)(includes o51 p223)

(waiting o52)
(includes o52 p34)(includes o52 p125)(includes o52 p147)(includes o52 p240)

(waiting o53)
(includes o53 p118)(includes o53 p212)(includes o53 p220)

(waiting o54)
(includes o54 p19)(includes o54 p66)(includes o54 p95)(includes o54 p134)(includes o54 p167)

(waiting o55)
(includes o55 p45)(includes o55 p161)(includes o55 p236)

(waiting o56)
(includes o56 p4)(includes o56 p5)(includes o56 p41)(includes o56 p47)(includes o56 p125)(includes o56 p128)(includes o56 p157)(includes o56 p178)(includes o56 p202)

(waiting o57)
(includes o57 p27)(includes o57 p90)(includes o57 p138)(includes o57 p165)(includes o57 p179)(includes o57 p223)(includes o57 p231)

(waiting o58)
(includes o58 p13)(includes o58 p67)(includes o58 p123)(includes o58 p127)(includes o58 p160)(includes o58 p163)(includes o58 p170)(includes o58 p180)(includes o58 p226)

(waiting o59)
(includes o59 p18)(includes o59 p29)(includes o59 p33)(includes o59 p112)(includes o59 p188)(includes o59 p194)(includes o59 p215)(includes o59 p240)

(waiting o60)
(includes o60 p3)(includes o60 p56)(includes o60 p64)(includes o60 p89)(includes o60 p93)(includes o60 p134)(includes o60 p156)(includes o60 p162)(includes o60 p165)(includes o60 p197)

(waiting o61)
(includes o61 p50)(includes o61 p131)(includes o61 p172)(includes o61 p231)

(waiting o62)
(includes o62 p3)(includes o62 p29)(includes o62 p44)(includes o62 p51)(includes o62 p52)(includes o62 p82)(includes o62 p110)(includes o62 p129)(includes o62 p184)(includes o62 p215)(includes o62 p240)

(waiting o63)
(includes o63 p33)(includes o63 p97)(includes o63 p187)(includes o63 p224)

(waiting o64)
(includes o64 p30)(includes o64 p100)(includes o64 p235)

(waiting o65)
(includes o65 p10)(includes o65 p21)(includes o65 p49)(includes o65 p59)(includes o65 p95)(includes o65 p108)(includes o65 p128)(includes o65 p146)(includes o65 p173)(includes o65 p191)(includes o65 p218)

(waiting o66)
(includes o66 p106)(includes o66 p137)

(waiting o67)
(includes o67 p17)(includes o67 p120)(includes o67 p121)(includes o67 p234)(includes o67 p243)

(waiting o68)
(includes o68 p4)(includes o68 p11)(includes o68 p104)(includes o68 p165)(includes o68 p175)

(waiting o69)
(includes o69 p18)(includes o69 p28)(includes o69 p96)(includes o69 p144)(includes o69 p162)(includes o69 p245)

(waiting o70)
(includes o70 p4)(includes o70 p23)(includes o70 p79)(includes o70 p82)(includes o70 p94)(includes o70 p119)(includes o70 p133)(includes o70 p142)(includes o70 p173)(includes o70 p244)(includes o70 p250)

(waiting o71)
(includes o71 p31)(includes o71 p58)(includes o71 p69)(includes o71 p139)(includes o71 p194)(includes o71 p219)

(waiting o72)
(includes o72 p35)(includes o72 p75)(includes o72 p85)(includes o72 p123)(includes o72 p209)(includes o72 p216)(includes o72 p234)

(waiting o73)
(includes o73 p47)(includes o73 p49)(includes o73 p132)(includes o73 p207)(includes o73 p247)

(waiting o74)
(includes o74 p8)(includes o74 p43)(includes o74 p44)(includes o74 p188)(includes o74 p203)(includes o74 p236)

(waiting o75)
(includes o75 p26)(includes o75 p98)(includes o75 p154)(includes o75 p198)(includes o75 p200)(includes o75 p227)

(waiting o76)
(includes o76 p25)(includes o76 p38)(includes o76 p65)(includes o76 p92)(includes o76 p122)

(waiting o77)
(includes o77 p8)(includes o77 p11)(includes o77 p79)(includes o77 p102)(includes o77 p167)(includes o77 p209)(includes o77 p238)

(waiting o78)
(includes o78 p139)

(waiting o79)
(includes o79 p17)(includes o79 p83)(includes o79 p149)(includes o79 p234)

(waiting o80)
(includes o80 p4)(includes o80 p18)(includes o80 p56)(includes o80 p89)(includes o80 p208)

(waiting o81)
(includes o81 p71)(includes o81 p88)(includes o81 p89)(includes o81 p119)(includes o81 p215)

(waiting o82)
(includes o82 p33)(includes o82 p42)(includes o82 p72)(includes o82 p205)(includes o82 p206)(includes o82 p220)(includes o82 p243)

(waiting o83)
(includes o83 p36)(includes o83 p142)(includes o83 p215)(includes o83 p246)

(waiting o84)
(includes o84 p1)(includes o84 p32)(includes o84 p44)(includes o84 p57)(includes o84 p70)(includes o84 p72)(includes o84 p133)(includes o84 p153)(includes o84 p164)(includes o84 p175)

(waiting o85)
(includes o85 p31)(includes o85 p82)(includes o85 p91)

(waiting o86)
(includes o86 p42)(includes o86 p59)(includes o86 p75)(includes o86 p101)(includes o86 p190)

(waiting o87)
(includes o87 p79)(includes o87 p82)(includes o87 p114)(includes o87 p138)(includes o87 p182)(includes o87 p183)(includes o87 p190)(includes o87 p194)

(waiting o88)
(includes o88 p1)(includes o88 p46)(includes o88 p50)(includes o88 p55)(includes o88 p79)(includes o88 p101)(includes o88 p178)(includes o88 p182)(includes o88 p207)

(waiting o89)
(includes o89 p27)(includes o89 p76)(includes o89 p90)(includes o89 p174)(includes o89 p200)(includes o89 p212)

(waiting o90)
(includes o90 p60)(includes o90 p121)(includes o90 p135)(includes o90 p218)

(waiting o91)
(includes o91 p48)(includes o91 p57)(includes o91 p67)(includes o91 p80)(includes o91 p112)(includes o91 p200)(includes o91 p245)

(waiting o92)
(includes o92 p20)(includes o92 p29)(includes o92 p66)(includes o92 p102)(includes o92 p118)(includes o92 p144)(includes o92 p175)

(waiting o93)
(includes o93 p4)(includes o93 p32)(includes o93 p56)(includes o93 p80)(includes o93 p120)

(waiting o94)
(includes o94 p30)(includes o94 p85)(includes o94 p97)(includes o94 p121)(includes o94 p132)(includes o94 p152)(includes o94 p177)(includes o94 p231)(includes o94 p239)

(waiting o95)
(includes o95 p11)(includes o95 p126)(includes o95 p151)(includes o95 p194)(includes o95 p212)

(waiting o96)
(includes o96 p6)(includes o96 p24)(includes o96 p34)(includes o96 p95)(includes o96 p96)(includes o96 p137)(includes o96 p170)(includes o96 p249)

(waiting o97)
(includes o97 p1)(includes o97 p63)(includes o97 p128)(includes o97 p135)(includes o97 p159)(includes o97 p163)(includes o97 p218)

(waiting o98)
(includes o98 p62)(includes o98 p64)(includes o98 p110)(includes o98 p174)(includes o98 p225)(includes o98 p247)

(waiting o99)
(includes o99 p15)(includes o99 p51)(includes o99 p59)(includes o99 p128)(includes o99 p182)(includes o99 p197)(includes o99 p206)(includes o99 p219)

(waiting o100)
(includes o100 p20)(includes o100 p49)(includes o100 p80)(includes o100 p107)(includes o100 p112)(includes o100 p141)(includes o100 p166)(includes o100 p173)(includes o100 p177)(includes o100 p243)

(waiting o101)
(includes o101 p77)(includes o101 p123)(includes o101 p200)(includes o101 p204)(includes o101 p209)

(waiting o102)
(includes o102 p27)(includes o102 p48)(includes o102 p91)(includes o102 p128)(includes o102 p133)(includes o102 p142)(includes o102 p160)(includes o102 p163)(includes o102 p220)(includes o102 p233)

(waiting o103)
(includes o103 p13)(includes o103 p38)(includes o103 p89)(includes o103 p180)(includes o103 p186)(includes o103 p187)(includes o103 p214)(includes o103 p233)(includes o103 p243)

(waiting o104)
(includes o104 p17)(includes o104 p30)(includes o104 p55)(includes o104 p91)(includes o104 p102)(includes o104 p197)(includes o104 p214)(includes o104 p243)

(waiting o105)
(includes o105 p35)(includes o105 p48)(includes o105 p72)(includes o105 p95)(includes o105 p163)(includes o105 p172)(includes o105 p192)

(waiting o106)
(includes o106 p2)(includes o106 p13)(includes o106 p32)(includes o106 p47)(includes o106 p86)(includes o106 p160)(includes o106 p198)(includes o106 p219)

(waiting o107)
(includes o107 p9)(includes o107 p173)(includes o107 p208)(includes o107 p220)

(waiting o108)
(includes o108 p102)(includes o108 p188)(includes o108 p204)(includes o108 p216)(includes o108 p225)

(waiting o109)
(includes o109 p67)(includes o109 p124)(includes o109 p145)(includes o109 p208)(includes o109 p238)

(waiting o110)
(includes o110 p32)(includes o110 p52)(includes o110 p113)(includes o110 p126)

(waiting o111)
(includes o111 p23)(includes o111 p26)(includes o111 p148)(includes o111 p159)(includes o111 p165)(includes o111 p178)(includes o111 p215)(includes o111 p225)

(waiting o112)
(includes o112 p8)(includes o112 p15)(includes o112 p17)(includes o112 p64)(includes o112 p84)(includes o112 p100)(includes o112 p147)(includes o112 p191)(includes o112 p211)

(waiting o113)
(includes o113 p12)(includes o113 p13)(includes o113 p24)(includes o113 p74)(includes o113 p80)(includes o113 p115)(includes o113 p117)(includes o113 p178)(includes o113 p249)

(waiting o114)
(includes o114 p13)(includes o114 p15)(includes o114 p36)(includes o114 p98)(includes o114 p167)(includes o114 p172)(includes o114 p189)(includes o114 p203)

(waiting o115)
(includes o115 p12)(includes o115 p36)(includes o115 p39)(includes o115 p55)(includes o115 p59)(includes o115 p91)(includes o115 p160)(includes o115 p179)(includes o115 p180)(includes o115 p206)

(waiting o116)
(includes o116 p66)(includes o116 p69)(includes o116 p107)(includes o116 p126)(includes o116 p132)(includes o116 p152)(includes o116 p161)(includes o116 p177)

(waiting o117)
(includes o117 p73)(includes o117 p119)(includes o117 p131)

(waiting o118)
(includes o118 p15)(includes o118 p18)(includes o118 p49)(includes o118 p69)(includes o118 p174)(includes o118 p177)(includes o118 p186)(includes o118 p215)(includes o118 p225)

(waiting o119)
(includes o119 p53)(includes o119 p78)(includes o119 p87)(includes o119 p150)(includes o119 p157)(includes o119 p166)(includes o119 p175)(includes o119 p223)

(waiting o120)
(includes o120 p4)(includes o120 p105)(includes o120 p121)(includes o120 p147)(includes o120 p191)

(waiting o121)
(includes o121 p26)(includes o121 p70)(includes o121 p85)(includes o121 p124)(includes o121 p135)(includes o121 p188)(includes o121 p207)(includes o121 p240)(includes o121 p247)

(waiting o122)
(includes o122 p6)(includes o122 p16)(includes o122 p81)(includes o122 p103)(includes o122 p130)(includes o122 p155)(includes o122 p195)(includes o122 p221)(includes o122 p225)

(waiting o123)
(includes o123 p28)(includes o123 p30)(includes o123 p64)(includes o123 p90)(includes o123 p215)

(waiting o124)
(includes o124 p2)(includes o124 p5)(includes o124 p16)(includes o124 p34)(includes o124 p68)(includes o124 p99)(includes o124 p100)(includes o124 p182)(includes o124 p188)(includes o124 p225)

(waiting o125)
(includes o125 p34)(includes o125 p125)(includes o125 p134)(includes o125 p214)

(waiting o126)
(includes o126 p28)(includes o126 p36)(includes o126 p56)(includes o126 p74)(includes o126 p128)(includes o126 p163)(includes o126 p167)(includes o126 p176)(includes o126 p179)(includes o126 p196)(includes o126 p240)

(waiting o127)
(includes o127 p38)(includes o127 p102)(includes o127 p143)(includes o127 p146)(includes o127 p178)(includes o127 p195)

(waiting o128)
(includes o128 p67)(includes o128 p102)(includes o128 p165)(includes o128 p213)(includes o128 p246)

(waiting o129)
(includes o129 p56)(includes o129 p67)(includes o129 p192)(includes o129 p245)

(waiting o130)
(includes o130 p3)(includes o130 p42)(includes o130 p112)(includes o130 p131)(includes o130 p166)(includes o130 p180)(includes o130 p201)

(waiting o131)
(includes o131 p96)

(waiting o132)
(includes o132 p19)(includes o132 p116)(includes o132 p136)(includes o132 p207)

(waiting o133)
(includes o133 p23)(includes o133 p37)(includes o133 p98)(includes o133 p143)

(waiting o134)
(includes o134 p40)(includes o134 p66)(includes o134 p78)(includes o134 p95)

(waiting o135)
(includes o135 p170)(includes o135 p194)

(waiting o136)
(includes o136 p52)(includes o136 p54)(includes o136 p184)(includes o136 p242)(includes o136 p247)

(waiting o137)
(includes o137 p1)(includes o137 p171)(includes o137 p194)

(waiting o138)
(includes o138 p59)(includes o138 p113)(includes o138 p140)(includes o138 p178)(includes o138 p200)(includes o138 p205)(includes o138 p221)(includes o138 p233)(includes o138 p241)

(waiting o139)
(includes o139 p141)(includes o139 p164)(includes o139 p175)(includes o139 p180)(includes o139 p202)(includes o139 p237)

(waiting o140)
(includes o140 p89)(includes o140 p96)(includes o140 p123)(includes o140 p143)(includes o140 p185)(includes o140 p240)

(waiting o141)
(includes o141 p67)(includes o141 p102)(includes o141 p137)(includes o141 p144)(includes o141 p150)(includes o141 p158)(includes o141 p227)

(waiting o142)
(includes o142 p139)(includes o142 p160)(includes o142 p178)(includes o142 p239)

(waiting o143)
(includes o143 p27)(includes o143 p28)(includes o143 p52)(includes o143 p97)(includes o143 p109)(includes o143 p118)(includes o143 p149)

(waiting o144)
(includes o144 p10)(includes o144 p14)(includes o144 p25)(includes o144 p29)(includes o144 p62)(includes o144 p64)(includes o144 p85)(includes o144 p94)(includes o144 p105)(includes o144 p168)(includes o144 p223)(includes o144 p224)

(waiting o145)
(includes o145 p43)(includes o145 p140)(includes o145 p208)(includes o145 p212)

(waiting o146)
(includes o146 p39)(includes o146 p45)(includes o146 p88)(includes o146 p181)(includes o146 p229)(includes o146 p240)

(waiting o147)
(includes o147 p62)(includes o147 p69)(includes o147 p85)(includes o147 p108)(includes o147 p155)(includes o147 p206)(includes o147 p216)(includes o147 p217)

(waiting o148)
(includes o148 p11)(includes o148 p22)(includes o148 p24)(includes o148 p59)(includes o148 p118)(includes o148 p201)(includes o148 p206)

(waiting o149)
(includes o149 p26)(includes o149 p198)(includes o149 p247)

(waiting o150)
(includes o150 p53)(includes o150 p81)(includes o150 p83)(includes o150 p130)(includes o150 p136)(includes o150 p217)

(waiting o151)
(includes o151 p42)(includes o151 p95)(includes o151 p113)(includes o151 p136)(includes o151 p140)(includes o151 p185)(includes o151 p220)(includes o151 p250)

(waiting o152)
(includes o152 p3)(includes o152 p11)(includes o152 p23)(includes o152 p24)(includes o152 p32)(includes o152 p75)(includes o152 p137)(includes o152 p170)(includes o152 p172)(includes o152 p174)(includes o152 p180)(includes o152 p203)

(waiting o153)
(includes o153 p144)(includes o153 p191)(includes o153 p236)

(waiting o154)
(includes o154 p7)(includes o154 p12)(includes o154 p23)(includes o154 p48)(includes o154 p69)(includes o154 p109)(includes o154 p166)

(waiting o155)
(includes o155 p4)(includes o155 p15)(includes o155 p17)(includes o155 p32)(includes o155 p41)(includes o155 p80)(includes o155 p225)

(waiting o156)
(includes o156 p68)(includes o156 p99)(includes o156 p137)(includes o156 p155)(includes o156 p188)

(waiting o157)
(includes o157 p10)(includes o157 p21)(includes o157 p80)(includes o157 p110)(includes o157 p111)(includes o157 p116)(includes o157 p118)(includes o157 p123)(includes o157 p136)(includes o157 p230)

(waiting o158)
(includes o158 p57)(includes o158 p88)(includes o158 p198)

(waiting o159)
(includes o159 p16)(includes o159 p17)(includes o159 p128)(includes o159 p135)(includes o159 p140)(includes o159 p149)(includes o159 p239)(includes o159 p245)

(waiting o160)
(includes o160 p118)(includes o160 p170)(includes o160 p178)(includes o160 p191)(includes o160 p249)

(waiting o161)
(includes o161 p9)(includes o161 p18)(includes o161 p115)(includes o161 p179)(includes o161 p202)

(waiting o162)
(includes o162 p26)(includes o162 p73)(includes o162 p109)(includes o162 p140)(includes o162 p160)(includes o162 p164)(includes o162 p200)(includes o162 p236)

(waiting o163)
(includes o163 p59)(includes o163 p83)(includes o163 p135)(includes o163 p164)(includes o163 p172)(includes o163 p186)(includes o163 p211)

(waiting o164)
(includes o164 p37)(includes o164 p98)(includes o164 p198)(includes o164 p204)

(waiting o165)
(includes o165 p23)(includes o165 p52)(includes o165 p134)(includes o165 p177)(includes o165 p228)

(waiting o166)
(includes o166 p61)(includes o166 p141)(includes o166 p151)

(waiting o167)
(includes o167 p87)(includes o167 p169)(includes o167 p195)(includes o167 p213)(includes o167 p219)

(waiting o168)
(includes o168 p38)(includes o168 p142)(includes o168 p158)(includes o168 p203)

(waiting o169)
(includes o169 p4)(includes o169 p34)(includes o169 p84)(includes o169 p115)(includes o169 p215)

(waiting o170)
(includes o170 p3)(includes o170 p41)(includes o170 p93)(includes o170 p111)(includes o170 p127)(includes o170 p161)(includes o170 p162)(includes o170 p173)(includes o170 p178)(includes o170 p203)(includes o170 p214)(includes o170 p232)

(waiting o171)
(includes o171 p8)(includes o171 p11)(includes o171 p54)

(waiting o172)
(includes o172 p41)(includes o172 p73)(includes o172 p79)(includes o172 p128)(includes o172 p176)(includes o172 p178)

(waiting o173)
(includes o173 p5)(includes o173 p8)(includes o173 p10)(includes o173 p60)(includes o173 p83)(includes o173 p133)(includes o173 p211)

(waiting o174)
(includes o174 p12)(includes o174 p108)(includes o174 p196)(includes o174 p201)

(waiting o175)
(includes o175 p20)(includes o175 p50)(includes o175 p65)(includes o175 p75)(includes o175 p119)(includes o175 p182)(includes o175 p224)(includes o175 p249)(includes o175 p250)

(waiting o176)
(includes o176 p19)(includes o176 p29)(includes o176 p44)(includes o176 p47)(includes o176 p54)(includes o176 p64)(includes o176 p73)(includes o176 p76)(includes o176 p80)(includes o176 p128)(includes o176 p161)(includes o176 p245)

(waiting o177)
(includes o177 p15)(includes o177 p66)(includes o177 p107)(includes o177 p150)(includes o177 p211)

(waiting o178)
(includes o178 p3)(includes o178 p7)(includes o178 p60)(includes o178 p162)(includes o178 p168)(includes o178 p184)(includes o178 p197)(includes o178 p219)

(waiting o179)
(includes o179 p28)(includes o179 p40)(includes o179 p43)(includes o179 p152)(includes o179 p206)(includes o179 p224)

(waiting o180)
(includes o180 p44)(includes o180 p97)(includes o180 p172)(includes o180 p179)

(waiting o181)
(includes o181 p11)(includes o181 p89)(includes o181 p179)(includes o181 p203)(includes o181 p224)

(waiting o182)
(includes o182 p209)

(waiting o183)
(includes o183 p3)(includes o183 p7)(includes o183 p11)(includes o183 p60)(includes o183 p72)(includes o183 p91)(includes o183 p116)(includes o183 p188)

(waiting o184)
(includes o184 p46)(includes o184 p76)(includes o184 p88)(includes o184 p134)

(waiting o185)
(includes o185 p9)(includes o185 p47)(includes o185 p96)(includes o185 p120)(includes o185 p193)(includes o185 p243)

(waiting o186)
(includes o186 p9)(includes o186 p21)(includes o186 p44)(includes o186 p164)(includes o186 p169)

(waiting o187)
(includes o187 p109)(includes o187 p119)(includes o187 p139)(includes o187 p226)(includes o187 p228)(includes o187 p248)

(waiting o188)
(includes o188 p20)(includes o188 p37)(includes o188 p45)(includes o188 p82)(includes o188 p94)(includes o188 p107)(includes o188 p196)(includes o188 p239)

(waiting o189)
(includes o189 p14)(includes o189 p44)(includes o189 p84)(includes o189 p119)(includes o189 p157)(includes o189 p193)

(waiting o190)
(includes o190 p41)(includes o190 p138)(includes o190 p155)(includes o190 p163)(includes o190 p174)(includes o190 p219)

(waiting o191)
(includes o191 p31)(includes o191 p35)(includes o191 p48)(includes o191 p139)(includes o191 p241)

(waiting o192)
(includes o192 p124)(includes o192 p169)(includes o192 p217)(includes o192 p250)

(waiting o193)
(includes o193 p73)(includes o193 p131)(includes o193 p166)(includes o193 p195)(includes o193 p200)(includes o193 p221)

(waiting o194)
(includes o194 p64)(includes o194 p71)(includes o194 p113)(includes o194 p115)(includes o194 p124)(includes o194 p163)

(waiting o195)
(includes o195 p17)(includes o195 p35)(includes o195 p88)(includes o195 p237)

(waiting o196)
(includes o196 p74)(includes o196 p123)(includes o196 p202)(includes o196 p208)(includes o196 p222)(includes o196 p243)

(waiting o197)
(includes o197 p2)(includes o197 p3)(includes o197 p114)(includes o197 p115)(includes o197 p129)(includes o197 p132)(includes o197 p134)(includes o197 p141)(includes o197 p169)(includes o197 p183)

(waiting o198)
(includes o198 p85)(includes o198 p97)(includes o198 p106)(includes o198 p186)(includes o198 p197)

(waiting o199)
(includes o199 p59)(includes o199 p91)(includes o199 p101)(includes o199 p138)(includes o199 p150)(includes o199 p188)(includes o199 p189)(includes o199 p202)(includes o199 p218)(includes o199 p226)(includes o199 p245)(includes o199 p249)

(waiting o200)
(includes o200 p59)(includes o200 p96)(includes o200 p112)(includes o200 p216)

(waiting o201)
(includes o201 p40)(includes o201 p181)(includes o201 p205)(includes o201 p236)(includes o201 p237)

(waiting o202)
(includes o202 p5)(includes o202 p66)(includes o202 p75)(includes o202 p85)(includes o202 p221)

(waiting o203)
(includes o203 p35)(includes o203 p38)(includes o203 p62)(includes o203 p67)(includes o203 p189)

(waiting o204)
(includes o204 p7)(includes o204 p25)(includes o204 p97)(includes o204 p99)(includes o204 p108)(includes o204 p171)(includes o204 p191)(includes o204 p215)(includes o204 p247)

(waiting o205)
(includes o205 p20)(includes o205 p242)

(waiting o206)
(includes o206 p44)(includes o206 p50)(includes o206 p93)(includes o206 p112)(includes o206 p133)(includes o206 p203)

(waiting o207)
(includes o207 p3)(includes o207 p34)(includes o207 p69)(includes o207 p112)(includes o207 p144)(includes o207 p154)(includes o207 p211)

(waiting o208)
(includes o208 p55)(includes o208 p83)(includes o208 p115)(includes o208 p142)(includes o208 p186)(includes o208 p195)(includes o208 p198)(includes o208 p225)(includes o208 p229)

(waiting o209)
(includes o209 p15)(includes o209 p53)(includes o209 p124)(includes o209 p167)(includes o209 p222)(includes o209 p232)

(waiting o210)
(includes o210 p70)(includes o210 p112)

(waiting o211)
(includes o211 p89)(includes o211 p96)(includes o211 p125)(includes o211 p195)(includes o211 p215)(includes o211 p218)(includes o211 p232)

(waiting o212)
(includes o212 p47)(includes o212 p70)(includes o212 p98)(includes o212 p107)(includes o212 p189)

(waiting o213)
(includes o213 p12)(includes o213 p32)(includes o213 p75)(includes o213 p108)(includes o213 p171)(includes o213 p236)

(waiting o214)
(includes o214 p26)(includes o214 p39)(includes o214 p40)(includes o214 p51)(includes o214 p59)(includes o214 p70)(includes o214 p109)(includes o214 p125)(includes o214 p164)(includes o214 p218)

(waiting o215)
(includes o215 p1)(includes o215 p34)(includes o215 p92)(includes o215 p107)(includes o215 p152)(includes o215 p154)(includes o215 p181)

(waiting o216)
(includes o216 p4)(includes o216 p24)(includes o216 p59)(includes o216 p69)(includes o216 p74)(includes o216 p178)(includes o216 p203)(includes o216 p204)(includes o216 p244)

(waiting o217)
(includes o217 p7)(includes o217 p27)(includes o217 p64)(includes o217 p168)(includes o217 p211)(includes o217 p242)

(waiting o218)
(includes o218 p36)(includes o218 p50)(includes o218 p94)

(waiting o219)
(includes o219 p51)(includes o219 p73)(includes o219 p79)(includes o219 p117)(includes o219 p247)

(waiting o220)
(includes o220 p70)(includes o220 p79)(includes o220 p112)

(waiting o221)
(includes o221 p76)(includes o221 p80)(includes o221 p148)(includes o221 p170)

(waiting o222)
(includes o222 p40)(includes o222 p213)(includes o222 p233)

(waiting o223)
(includes o223 p1)(includes o223 p30)(includes o223 p40)(includes o223 p111)(includes o223 p144)(includes o223 p230)

(waiting o224)
(includes o224 p6)(includes o224 p11)(includes o224 p35)(includes o224 p74)(includes o224 p89)(includes o224 p107)

(waiting o225)
(includes o225 p83)(includes o225 p106)(includes o225 p124)(includes o225 p214)(includes o225 p236)

(waiting o226)
(includes o226 p2)(includes o226 p137)(includes o226 p153)(includes o226 p203)(includes o226 p228)

(waiting o227)
(includes o227 p7)(includes o227 p38)(includes o227 p79)(includes o227 p182)(includes o227 p189)(includes o227 p232)(includes o227 p242)

(waiting o228)
(includes o228 p53)(includes o228 p115)(includes o228 p126)(includes o228 p152)(includes o228 p182)(includes o228 p204)

(waiting o229)
(includes o229 p86)(includes o229 p103)(includes o229 p122)(includes o229 p181)(includes o229 p191)

(waiting o230)
(includes o230 p26)(includes o230 p31)(includes o230 p108)(includes o230 p111)(includes o230 p124)(includes o230 p145)(includes o230 p158)(includes o230 p169)(includes o230 p173)(includes o230 p216)

(waiting o231)
(includes o231 p29)(includes o231 p47)(includes o231 p55)(includes o231 p92)(includes o231 p104)(includes o231 p132)(includes o231 p200)(includes o231 p238)(includes o231 p249)

(waiting o232)
(includes o232 p10)(includes o232 p38)(includes o232 p40)(includes o232 p125)(includes o232 p127)(includes o232 p213)(includes o232 p214)

(waiting o233)
(includes o233 p6)(includes o233 p8)(includes o233 p10)(includes o233 p27)(includes o233 p44)(includes o233 p208)(includes o233 p235)(includes o233 p244)

(waiting o234)
(includes o234 p70)(includes o234 p100)

(waiting o235)
(includes o235 p74)(includes o235 p86)(includes o235 p90)(includes o235 p117)(includes o235 p148)(includes o235 p158)(includes o235 p172)(includes o235 p199)

(waiting o236)
(includes o236 p25)(includes o236 p49)(includes o236 p65)(includes o236 p90)(includes o236 p105)(includes o236 p187)(includes o236 p215)(includes o236 p226)(includes o236 p233)(includes o236 p246)

(waiting o237)
(includes o237 p12)(includes o237 p18)(includes o237 p36)(includes o237 p38)(includes o237 p110)(includes o237 p180)(includes o237 p215)

(waiting o238)
(includes o238 p1)(includes o238 p41)(includes o238 p46)(includes o238 p107)(includes o238 p123)(includes o238 p146)(includes o238 p205)

(waiting o239)
(includes o239 p10)(includes o239 p239)(includes o239 p240)

(waiting o240)
(includes o240 p35)(includes o240 p55)(includes o240 p58)(includes o240 p113)(includes o240 p118)(includes o240 p237)(includes o240 p248)

(waiting o241)
(includes o241 p8)(includes o241 p166)(includes o241 p189)(includes o241 p198)

(waiting o242)
(includes o242 p16)(includes o242 p89)(includes o242 p105)(includes o242 p148)(includes o242 p185)(includes o242 p202)(includes o242 p204)(includes o242 p214)

(waiting o243)
(includes o243 p15)(includes o243 p24)(includes o243 p59)(includes o243 p89)(includes o243 p106)(includes o243 p170)(includes o243 p210)(includes o243 p211)(includes o243 p213)(includes o243 p223)

(waiting o244)
(includes o244 p17)(includes o244 p29)(includes o244 p46)(includes o244 p59)(includes o244 p105)(includes o244 p109)(includes o244 p112)

(waiting o245)
(includes o245 p14)(includes o245 p22)(includes o245 p45)(includes o245 p113)(includes o245 p161)(includes o245 p196)(includes o245 p221)

(waiting o246)
(includes o246 p47)(includes o246 p106)(includes o246 p109)(includes o246 p190)(includes o246 p229)(includes o246 p238)(includes o246 p241)

(waiting o247)
(includes o247 p97)(includes o247 p225)(includes o247 p250)

(waiting o248)
(includes o248 p25)(includes o248 p162)(includes o248 p164)(includes o248 p174)(includes o248 p185)(includes o248 p225)

(waiting o249)
(includes o249 p45)(includes o249 p73)(includes o249 p124)(includes o249 p142)(includes o249 p205)(includes o249 p246)

(waiting o250)
(includes o250 p20)(includes o250 p97)(includes o250 p105)(includes o250 p158)(includes o250 p227)(includes o250 p236)

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
))

(:metric minimize (total-cost))

)

