(define (problem os-sequencedstrips-p250_2)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210 n211 n212 n213 n214 n215 n216 n217 n218 n219 n220 n221 n222 n223 n224 n225 n226 n227 n228 n229 n230 n231 n232 n233 n234 n235 n236 n237 n238 n239 n240 n241 n242 n243 n244 n245 n246 n247 n248 n249 n250  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) (next-count n210 n211) (next-count n211 n212) (next-count n212 n213) (next-count n213 n214) (next-count n214 n215) (next-count n215 n216) (next-count n216 n217) (next-count n217 n218) (next-count n218 n219) (next-count n219 n220) (next-count n220 n221) (next-count n221 n222) (next-count n222 n223) (next-count n223 n224) (next-count n224 n225) (next-count n225 n226) (next-count n226 n227) (next-count n227 n228) (next-count n228 n229) (next-count n229 n230) (next-count n230 n231) (next-count n231 n232) (next-count n232 n233) (next-count n233 n234) (next-count n234 n235) (next-count n235 n236) (next-count n236 n237) (next-count n237 n238) (next-count n238 n239) (next-count n239 n240) (next-count n240 n241) (next-count n241 n242) (next-count n242 n243) (next-count n243 n244) (next-count n244 n245) (next-count n245 n246) (next-count n246 n247) (next-count n247 n248) (next-count n248 n249) (next-count n249 n250) 
(stacks-avail n0)

(waiting o1)
(includes o1 p25)(includes o1 p71)(includes o1 p100)

(waiting o2)
(includes o2 p9)(includes o2 p80)(includes o2 p93)(includes o2 p126)(includes o2 p133)(includes o2 p138)(includes o2 p149)

(waiting o3)
(includes o3 p20)(includes o3 p202)(includes o3 p204)

(waiting o4)
(includes o4 p13)(includes o4 p238)

(waiting o5)
(includes o5 p40)(includes o5 p101)(includes o5 p118)(includes o5 p126)(includes o5 p132)(includes o5 p185)(includes o5 p191)(includes o5 p203)(includes o5 p242)

(waiting o6)
(includes o6 p29)(includes o6 p33)(includes o6 p34)(includes o6 p61)(includes o6 p70)

(waiting o7)
(includes o7 p13)(includes o7 p77)(includes o7 p105)(includes o7 p156)(includes o7 p187)(includes o7 p223)

(waiting o8)
(includes o8 p10)(includes o8 p35)(includes o8 p83)(includes o8 p84)(includes o8 p130)(includes o8 p137)(includes o8 p179)(includes o8 p242)

(waiting o9)
(includes o9 p3)(includes o9 p8)(includes o9 p13)(includes o9 p60)(includes o9 p68)(includes o9 p103)(includes o9 p121)

(waiting o10)
(includes o10 p3)(includes o10 p77)(includes o10 p92)(includes o10 p127)(includes o10 p132)(includes o10 p163)(includes o10 p200)

(waiting o11)
(includes o11 p62)(includes o11 p180)(includes o11 p183)(includes o11 p207)(includes o11 p248)

(waiting o12)
(includes o12 p22)(includes o12 p46)(includes o12 p97)(includes o12 p122)(includes o12 p197)(includes o12 p225)(includes o12 p229)(includes o12 p234)

(waiting o13)
(includes o13 p53)(includes o13 p63)(includes o13 p152)(includes o13 p240)

(waiting o14)
(includes o14 p35)(includes o14 p53)(includes o14 p80)(includes o14 p104)(includes o14 p124)(includes o14 p158)(includes o14 p165)(includes o14 p191)

(waiting o15)
(includes o15 p6)(includes o15 p53)(includes o15 p67)(includes o15 p162)(includes o15 p209)(includes o15 p218)

(waiting o16)
(includes o16 p65)(includes o16 p130)(includes o16 p154)(includes o16 p203)(includes o16 p228)

(waiting o17)
(includes o17 p18)(includes o17 p86)(includes o17 p145)(includes o17 p159)(includes o17 p171)(includes o17 p183)(includes o17 p244)(includes o17 p249)

(waiting o18)
(includes o18 p18)(includes o18 p27)(includes o18 p68)(includes o18 p135)(includes o18 p159)(includes o18 p177)(includes o18 p235)

(waiting o19)
(includes o19 p61)(includes o19 p113)(includes o19 p120)(includes o19 p144)(includes o19 p198)(includes o19 p245)(includes o19 p246)

(waiting o20)
(includes o20 p37)(includes o20 p61)(includes o20 p205)(includes o20 p209)

(waiting o21)
(includes o21 p51)(includes o21 p96)(includes o21 p98)(includes o21 p116)(includes o21 p147)(includes o21 p161)(includes o21 p162)(includes o21 p174)(includes o21 p181)(includes o21 p189)(includes o21 p241)

(waiting o22)
(includes o22 p35)(includes o22 p36)(includes o22 p111)(includes o22 p139)

(waiting o23)
(includes o23 p14)(includes o23 p19)(includes o23 p41)(includes o23 p68)(includes o23 p69)(includes o23 p105)(includes o23 p135)(includes o23 p147)(includes o23 p178)(includes o23 p190)(includes o23 p195)(includes o23 p206)

(waiting o24)
(includes o24 p12)(includes o24 p13)(includes o24 p89)(includes o24 p91)(includes o24 p174)(includes o24 p191)(includes o24 p199)(includes o24 p248)

(waiting o25)
(includes o25 p141)(includes o25 p159)(includes o25 p168)(includes o25 p184)(includes o25 p223)

(waiting o26)
(includes o26 p64)(includes o26 p94)(includes o26 p101)(includes o26 p195)

(waiting o27)
(includes o27 p23)(includes o27 p32)(includes o27 p34)(includes o27 p41)(includes o27 p53)(includes o27 p56)(includes o27 p164)(includes o27 p166)

(waiting o28)
(includes o28 p16)(includes o28 p57)(includes o28 p107)(includes o28 p139)(includes o28 p244)

(waiting o29)
(includes o29 p184)(includes o29 p218)(includes o29 p244)

(waiting o30)
(includes o30 p66)(includes o30 p98)(includes o30 p100)(includes o30 p107)(includes o30 p132)(includes o30 p155)(includes o30 p184)(includes o30 p190)(includes o30 p194)

(waiting o31)
(includes o31 p7)(includes o31 p42)(includes o31 p66)(includes o31 p82)(includes o31 p103)(includes o31 p117)(includes o31 p183)(includes o31 p198)

(waiting o32)
(includes o32 p36)(includes o32 p75)(includes o32 p92)(includes o32 p101)(includes o32 p115)(includes o32 p118)(includes o32 p157)(includes o32 p204)(includes o32 p211)(includes o32 p214)(includes o32 p236)

(waiting o33)
(includes o33 p31)(includes o33 p35)(includes o33 p87)(includes o33 p137)(includes o33 p186)

(waiting o34)
(includes o34 p23)(includes o34 p48)(includes o34 p55)(includes o34 p130)(includes o34 p235)

(waiting o35)
(includes o35 p53)(includes o35 p178)(includes o35 p199)(includes o35 p211)

(waiting o36)
(includes o36 p20)(includes o36 p133)(includes o36 p147)(includes o36 p156)(includes o36 p204)(includes o36 p206)

(waiting o37)
(includes o37 p44)(includes o37 p76)(includes o37 p85)(includes o37 p123)(includes o37 p135)(includes o37 p168)(includes o37 p171)(includes o37 p180)(includes o37 p213)(includes o37 p214)(includes o37 p237)

(waiting o38)
(includes o38 p46)(includes o38 p92)(includes o38 p104)(includes o38 p129)(includes o38 p144)

(waiting o39)
(includes o39 p32)(includes o39 p42)(includes o39 p63)(includes o39 p78)(includes o39 p98)(includes o39 p124)(includes o39 p135)(includes o39 p187)(includes o39 p188)(includes o39 p200)(includes o39 p229)

(waiting o40)
(includes o40 p101)(includes o40 p137)(includes o40 p199)(includes o40 p236)

(waiting o41)
(includes o41 p135)(includes o41 p169)(includes o41 p177)(includes o41 p194)(includes o41 p213)(includes o41 p214)(includes o41 p247)

(waiting o42)
(includes o42 p92)(includes o42 p184)

(waiting o43)
(includes o43 p6)(includes o43 p126)(includes o43 p144)(includes o43 p179)(includes o43 p199)

(waiting o44)
(includes o44 p59)(includes o44 p168)(includes o44 p250)

(waiting o45)
(includes o45 p65)(includes o45 p112)(includes o45 p156)(includes o45 p175)(includes o45 p222)

(waiting o46)
(includes o46 p9)(includes o46 p56)(includes o46 p61)(includes o46 p70)(includes o46 p85)

(waiting o47)
(includes o47 p131)(includes o47 p150)(includes o47 p185)(includes o47 p188)(includes o47 p236)

(waiting o48)
(includes o48 p78)(includes o48 p171)(includes o48 p245)

(waiting o49)
(includes o49 p27)(includes o49 p89)(includes o49 p91)(includes o49 p221)

(waiting o50)
(includes o50 p2)(includes o50 p4)(includes o50 p35)(includes o50 p143)(includes o50 p178)(includes o50 p211)(includes o50 p225)(includes o50 p230)(includes o50 p249)

(waiting o51)
(includes o51 p179)(includes o51 p202)(includes o51 p246)

(waiting o52)
(includes o52 p74)(includes o52 p130)

(waiting o53)
(includes o53 p131)(includes o53 p150)(includes o53 p164)(includes o53 p244)(includes o53 p246)

(waiting o54)
(includes o54 p94)(includes o54 p151)(includes o54 p223)(includes o54 p243)

(waiting o55)
(includes o55 p2)(includes o55 p9)(includes o55 p10)(includes o55 p48)(includes o55 p111)(includes o55 p166)(includes o55 p183)(includes o55 p198)(includes o55 p232)(includes o55 p237)

(waiting o56)
(includes o56 p46)(includes o56 p150)(includes o56 p158)(includes o56 p172)(includes o56 p179)(includes o56 p180)

(waiting o57)
(includes o57 p53)(includes o57 p54)(includes o57 p127)(includes o57 p133)(includes o57 p191)(includes o57 p231)(includes o57 p247)

(waiting o58)
(includes o58 p7)(includes o58 p16)(includes o58 p27)(includes o58 p79)(includes o58 p142)(includes o58 p163)(includes o58 p182)(includes o58 p213)(includes o58 p232)

(waiting o59)
(includes o59 p31)(includes o59 p39)(includes o59 p58)(includes o59 p106)(includes o59 p128)(includes o59 p129)(includes o59 p194)(includes o59 p218)

(waiting o60)
(includes o60 p140)

(waiting o61)
(includes o61 p18)(includes o61 p30)(includes o61 p40)(includes o61 p93)(includes o61 p139)(includes o61 p158)(includes o61 p211)(includes o61 p240)

(waiting o62)
(includes o62 p22)(includes o62 p23)(includes o62 p93)(includes o62 p111)(includes o62 p172)(includes o62 p179)(includes o62 p184)(includes o62 p201)(includes o62 p226)(includes o62 p242)

(waiting o63)
(includes o63 p26)(includes o63 p65)(includes o63 p82)(includes o63 p99)(includes o63 p139)

(waiting o64)
(includes o64 p51)(includes o64 p166)(includes o64 p173)(includes o64 p174)(includes o64 p184)

(waiting o65)
(includes o65 p37)(includes o65 p40)(includes o65 p90)(includes o65 p115)(includes o65 p168)(includes o65 p183)(includes o65 p203)

(waiting o66)
(includes o66 p29)(includes o66 p48)(includes o66 p60)(includes o66 p87)(includes o66 p89)(includes o66 p152)(includes o66 p181)(includes o66 p206)(includes o66 p209)(includes o66 p239)

(waiting o67)
(includes o67 p7)(includes o67 p12)(includes o67 p76)(includes o67 p83)(includes o67 p108)(includes o67 p170)(includes o67 p201)(includes o67 p206)(includes o67 p249)

(waiting o68)
(includes o68 p24)(includes o68 p65)(includes o68 p122)(includes o68 p131)(includes o68 p162)(includes o68 p222)

(waiting o69)
(includes o69 p48)(includes o69 p63)(includes o69 p156)(includes o69 p185)(includes o69 p191)(includes o69 p198)(includes o69 p235)

(waiting o70)
(includes o70 p3)(includes o70 p103)(includes o70 p109)(includes o70 p143)(includes o70 p180)(includes o70 p196)(includes o70 p222)

(waiting o71)
(includes o71 p35)(includes o71 p39)(includes o71 p46)(includes o71 p68)(includes o71 p92)(includes o71 p99)(includes o71 p142)(includes o71 p250)

(waiting o72)
(includes o72 p19)(includes o72 p24)(includes o72 p47)(includes o72 p48)(includes o72 p64)(includes o72 p77)(includes o72 p109)(includes o72 p125)(includes o72 p147)(includes o72 p155)(includes o72 p188)

(waiting o73)
(includes o73 p100)(includes o73 p136)(includes o73 p159)(includes o73 p180)(includes o73 p230)

(waiting o74)
(includes o74 p4)(includes o74 p19)(includes o74 p91)(includes o74 p168)(includes o74 p193)(includes o74 p217)(includes o74 p245)

(waiting o75)
(includes o75 p49)(includes o75 p88)(includes o75 p92)(includes o75 p101)(includes o75 p118)(includes o75 p136)(includes o75 p211)

(waiting o76)
(includes o76 p16)(includes o76 p26)(includes o76 p38)(includes o76 p57)(includes o76 p75)(includes o76 p86)(includes o76 p109)(includes o76 p139)(includes o76 p159)(includes o76 p188)(includes o76 p227)

(waiting o77)
(includes o77 p27)(includes o77 p48)(includes o77 p90)(includes o77 p223)(includes o77 p234)

(waiting o78)
(includes o78 p11)(includes o78 p43)(includes o78 p50)(includes o78 p173)(includes o78 p176)

(waiting o79)
(includes o79 p104)(includes o79 p160)(includes o79 p239)

(waiting o80)
(includes o80 p10)(includes o80 p83)(includes o80 p165)(includes o80 p202)(includes o80 p205)(includes o80 p250)

(waiting o81)
(includes o81 p18)(includes o81 p39)(includes o81 p45)(includes o81 p55)(includes o81 p131)(includes o81 p151)(includes o81 p181)(includes o81 p200)(includes o81 p204)(includes o81 p223)(includes o81 p250)

(waiting o82)
(includes o82 p30)(includes o82 p110)(includes o82 p135)(includes o82 p215)

(waiting o83)
(includes o83 p18)(includes o83 p145)(includes o83 p146)(includes o83 p147)(includes o83 p156)(includes o83 p169)

(waiting o84)
(includes o84 p13)(includes o84 p21)(includes o84 p47)(includes o84 p150)(includes o84 p172)(includes o84 p228)

(waiting o85)
(includes o85 p5)(includes o85 p12)(includes o85 p74)(includes o85 p115)

(waiting o86)
(includes o86 p11)(includes o86 p21)(includes o86 p41)(includes o86 p51)(includes o86 p89)(includes o86 p97)(includes o86 p101)(includes o86 p108)(includes o86 p115)(includes o86 p157)(includes o86 p160)(includes o86 p199)

(waiting o87)
(includes o87 p1)(includes o87 p53)(includes o87 p131)

(waiting o88)
(includes o88 p87)(includes o88 p108)(includes o88 p137)(includes o88 p201)(includes o88 p219)

(waiting o89)
(includes o89 p4)(includes o89 p80)(includes o89 p89)(includes o89 p108)(includes o89 p121)(includes o89 p148)(includes o89 p169)(includes o89 p204)

(waiting o90)
(includes o90 p58)(includes o90 p95)(includes o90 p118)(includes o90 p136)(includes o90 p177)(includes o90 p188)(includes o90 p230)

(waiting o91)
(includes o91 p2)(includes o91 p3)(includes o91 p21)(includes o91 p25)(includes o91 p26)(includes o91 p41)(includes o91 p56)(includes o91 p62)(includes o91 p78)(includes o91 p121)(includes o91 p143)

(waiting o92)
(includes o92 p49)(includes o92 p96)(includes o92 p98)(includes o92 p202)(includes o92 p227)

(waiting o93)
(includes o93 p58)(includes o93 p112)(includes o93 p117)(includes o93 p122)(includes o93 p162)(includes o93 p170)(includes o93 p219)(includes o93 p244)

(waiting o94)
(includes o94 p86)(includes o94 p128)

(waiting o95)
(includes o95 p7)(includes o95 p106)

(waiting o96)
(includes o96 p82)(includes o96 p97)(includes o96 p105)(includes o96 p119)(includes o96 p130)(includes o96 p151)(includes o96 p156)(includes o96 p181)(includes o96 p222)

(waiting o97)
(includes o97 p56)(includes o97 p107)(includes o97 p110)(includes o97 p113)(includes o97 p134)(includes o97 p155)(includes o97 p227)(includes o97 p244)

(waiting o98)
(includes o98 p7)(includes o98 p24)(includes o98 p45)(includes o98 p166)(includes o98 p189)(includes o98 p191)(includes o98 p217)

(waiting o99)
(includes o99 p4)(includes o99 p12)(includes o99 p17)(includes o99 p19)(includes o99 p104)(includes o99 p201)

(waiting o100)
(includes o100 p56)(includes o100 p116)(includes o100 p134)(includes o100 p138)(includes o100 p153)(includes o100 p184)(includes o100 p229)

(waiting o101)
(includes o101 p11)(includes o101 p80)(includes o101 p175)(includes o101 p209)(includes o101 p246)

(waiting o102)
(includes o102 p70)(includes o102 p89)(includes o102 p115)(includes o102 p131)(includes o102 p236)

(waiting o103)
(includes o103 p4)(includes o103 p21)(includes o103 p39)(includes o103 p76)(includes o103 p125)(includes o103 p224)

(waiting o104)
(includes o104 p228)(includes o104 p243)

(waiting o105)
(includes o105 p18)(includes o105 p19)(includes o105 p20)(includes o105 p25)(includes o105 p37)(includes o105 p71)(includes o105 p133)(includes o105 p169)(includes o105 p247)(includes o105 p248)

(waiting o106)
(includes o106 p123)(includes o106 p223)(includes o106 p225)(includes o106 p228)(includes o106 p234)

(waiting o107)
(includes o107 p26)(includes o107 p27)(includes o107 p53)(includes o107 p95)(includes o107 p111)(includes o107 p180)(includes o107 p189)

(waiting o108)
(includes o108 p28)(includes o108 p33)(includes o108 p92)(includes o108 p142)(includes o108 p144)(includes o108 p170)

(waiting o109)
(includes o109 p61)(includes o109 p91)(includes o109 p132)(includes o109 p160)

(waiting o110)
(includes o110 p24)(includes o110 p49)(includes o110 p85)(includes o110 p113)(includes o110 p129)(includes o110 p155)(includes o110 p159)(includes o110 p169)(includes o110 p197)(includes o110 p242)

(waiting o111)
(includes o111 p1)(includes o111 p32)(includes o111 p56)(includes o111 p64)(includes o111 p84)(includes o111 p157)(includes o111 p182)(includes o111 p188)

(waiting o112)
(includes o112 p111)(includes o112 p180)(includes o112 p227)(includes o112 p245)

(waiting o113)
(includes o113 p22)(includes o113 p99)(includes o113 p162)(includes o113 p184)(includes o113 p193)(includes o113 p226)(includes o113 p232)

(waiting o114)
(includes o114 p57)(includes o114 p225)

(waiting o115)
(includes o115 p5)(includes o115 p37)(includes o115 p101)

(waiting o116)
(includes o116 p56)(includes o116 p84)(includes o116 p101)(includes o116 p133)(includes o116 p233)

(waiting o117)
(includes o117 p105)(includes o117 p147)(includes o117 p167)(includes o117 p188)(includes o117 p206)

(waiting o118)
(includes o118 p38)(includes o118 p46)(includes o118 p160)(includes o118 p175)(includes o118 p198)(includes o118 p201)(includes o118 p213)(includes o118 p223)

(waiting o119)
(includes o119 p178)(includes o119 p203)(includes o119 p239)

(waiting o120)
(includes o120 p16)(includes o120 p34)(includes o120 p36)(includes o120 p78)

(waiting o121)
(includes o121 p31)(includes o121 p146)(includes o121 p187)(includes o121 p217)(includes o121 p234)(includes o121 p241)

(waiting o122)
(includes o122 p34)(includes o122 p83)(includes o122 p116)(includes o122 p131)(includes o122 p178)(includes o122 p179)(includes o122 p223)

(waiting o123)
(includes o123 p184)(includes o123 p240)

(waiting o124)
(includes o124 p39)(includes o124 p113)(includes o124 p189)(includes o124 p207)(includes o124 p238)(includes o124 p239)(includes o124 p240)(includes o124 p245)

(waiting o125)
(includes o125 p66)(includes o125 p78)(includes o125 p85)(includes o125 p86)(includes o125 p105)(includes o125 p138)(includes o125 p140)(includes o125 p143)(includes o125 p167)(includes o125 p238)(includes o125 p246)

(waiting o126)
(includes o126 p31)(includes o126 p68)(includes o126 p111)(includes o126 p130)(includes o126 p193)(includes o126 p237)

(waiting o127)
(includes o127 p169)(includes o127 p180)(includes o127 p232)

(waiting o128)
(includes o128 p52)(includes o128 p53)(includes o128 p231)(includes o128 p243)

(waiting o129)
(includes o129 p68)(includes o129 p78)(includes o129 p118)(includes o129 p134)(includes o129 p153)(includes o129 p159)(includes o129 p170)(includes o129 p248)

(waiting o130)
(includes o130 p2)(includes o130 p63)(includes o130 p139)(includes o130 p161)(includes o130 p181)(includes o130 p211)

(waiting o131)
(includes o131 p11)(includes o131 p55)(includes o131 p70)(includes o131 p117)(includes o131 p126)

(waiting o132)
(includes o132 p53)(includes o132 p89)(includes o132 p140)(includes o132 p185)

(waiting o133)
(includes o133 p1)(includes o133 p2)(includes o133 p49)(includes o133 p55)(includes o133 p62)(includes o133 p89)(includes o133 p140)(includes o133 p171)(includes o133 p214)(includes o133 p218)

(waiting o134)
(includes o134 p47)(includes o134 p107)(includes o134 p170)(includes o134 p196)(includes o134 p224)(includes o134 p225)

(waiting o135)
(includes o135 p64)(includes o135 p122)(includes o135 p150)(includes o135 p221)

(waiting o136)
(includes o136 p10)(includes o136 p26)(includes o136 p109)(includes o136 p148)(includes o136 p179)(includes o136 p182)(includes o136 p199)(includes o136 p202)(includes o136 p244)

(waiting o137)
(includes o137 p4)(includes o137 p20)(includes o137 p69)(includes o137 p131)(includes o137 p158)(includes o137 p163)(includes o137 p197)(includes o137 p243)

(waiting o138)
(includes o138 p26)(includes o138 p150)(includes o138 p170)(includes o138 p207)

(waiting o139)
(includes o139 p78)(includes o139 p87)(includes o139 p123)(includes o139 p247)

(waiting o140)
(includes o140 p11)(includes o140 p27)(includes o140 p71)(includes o140 p78)(includes o140 p87)(includes o140 p109)(includes o140 p139)

(waiting o141)
(includes o141 p7)(includes o141 p29)(includes o141 p85)(includes o141 p136)(includes o141 p211)(includes o141 p237)(includes o141 p240)

(waiting o142)
(includes o142 p11)(includes o142 p18)(includes o142 p182)(includes o142 p189)(includes o142 p236)

(waiting o143)
(includes o143 p140)(includes o143 p196)(includes o143 p210)(includes o143 p244)

(waiting o144)
(includes o144 p48)(includes o144 p151)(includes o144 p219)(includes o144 p234)

(waiting o145)
(includes o145 p36)(includes o145 p79)(includes o145 p119)(includes o145 p149)(includes o145 p162)(includes o145 p187)(includes o145 p201)

(waiting o146)
(includes o146 p34)(includes o146 p60)(includes o146 p77)(includes o146 p166)(includes o146 p176)(includes o146 p185)(includes o146 p188)(includes o146 p193)

(waiting o147)
(includes o147 p16)(includes o147 p106)

(waiting o148)
(includes o148 p77)(includes o148 p125)(includes o148 p205)

(waiting o149)
(includes o149 p136)(includes o149 p139)(includes o149 p179)

(waiting o150)
(includes o150 p82)(includes o150 p93)(includes o150 p174)(includes o150 p242)

(waiting o151)
(includes o151 p6)(includes o151 p33)(includes o151 p40)(includes o151 p75)(includes o151 p81)(includes o151 p85)(includes o151 p113)(includes o151 p137)(includes o151 p238)

(waiting o152)
(includes o152 p25)(includes o152 p201)(includes o152 p240)

(waiting o153)
(includes o153 p108)(includes o153 p144)(includes o153 p178)(includes o153 p221)

(waiting o154)
(includes o154 p13)(includes o154 p51)(includes o154 p62)(includes o154 p94)(includes o154 p96)(includes o154 p169)(includes o154 p173)(includes o154 p176)(includes o154 p211)

(waiting o155)
(includes o155 p24)(includes o155 p68)(includes o155 p130)(includes o155 p151)

(waiting o156)
(includes o156 p26)(includes o156 p49)(includes o156 p83)(includes o156 p92)(includes o156 p94)(includes o156 p170)

(waiting o157)
(includes o157 p1)(includes o157 p35)(includes o157 p42)(includes o157 p56)(includes o157 p122)(includes o157 p143)

(waiting o158)
(includes o158 p42)(includes o158 p47)(includes o158 p70)(includes o158 p88)(includes o158 p117)(includes o158 p127)

(waiting o159)
(includes o159 p32)(includes o159 p61)(includes o159 p84)(includes o159 p97)(includes o159 p135)(includes o159 p143)(includes o159 p156)(includes o159 p192)(includes o159 p215)(includes o159 p222)

(waiting o160)
(includes o160 p76)(includes o160 p116)(includes o160 p130)(includes o160 p155)

(waiting o161)
(includes o161 p25)(includes o161 p34)(includes o161 p38)(includes o161 p44)(includes o161 p53)(includes o161 p137)(includes o161 p170)(includes o161 p206)(includes o161 p218)(includes o161 p226)

(waiting o162)
(includes o162 p12)(includes o162 p223)(includes o162 p235)(includes o162 p245)

(waiting o163)
(includes o163 p6)(includes o163 p12)(includes o163 p16)(includes o163 p62)(includes o163 p130)(includes o163 p184)(includes o163 p242)

(waiting o164)
(includes o164 p28)(includes o164 p104)(includes o164 p196)(includes o164 p201)

(waiting o165)
(includes o165 p73)(includes o165 p173)(includes o165 p202)(includes o165 p241)

(waiting o166)
(includes o166 p5)(includes o166 p42)(includes o166 p43)(includes o166 p222)

(waiting o167)
(includes o167 p28)(includes o167 p33)(includes o167 p36)(includes o167 p57)(includes o167 p78)(includes o167 p85)(includes o167 p100)(includes o167 p152)(includes o167 p177)(includes o167 p209)(includes o167 p220)

(waiting o168)
(includes o168 p55)(includes o168 p89)(includes o168 p141)(includes o168 p143)(includes o168 p153)(includes o168 p211)(includes o168 p243)

(waiting o169)
(includes o169 p1)(includes o169 p8)(includes o169 p63)(includes o169 p68)(includes o169 p93)(includes o169 p158)(includes o169 p163)(includes o169 p166)

(waiting o170)
(includes o170 p14)(includes o170 p49)(includes o170 p93)(includes o170 p189)

(waiting o171)
(includes o171 p50)

(waiting o172)
(includes o172 p67)(includes o172 p117)(includes o172 p202)(includes o172 p222)

(waiting o173)
(includes o173 p138)(includes o173 p169)(includes o173 p190)(includes o173 p197)(includes o173 p246)

(waiting o174)
(includes o174 p1)(includes o174 p76)(includes o174 p79)(includes o174 p111)(includes o174 p119)(includes o174 p141)(includes o174 p202)(includes o174 p223)(includes o174 p224)

(waiting o175)
(includes o175 p14)(includes o175 p21)(includes o175 p33)(includes o175 p90)(includes o175 p121)(includes o175 p123)(includes o175 p131)(includes o175 p132)(includes o175 p178)(includes o175 p195)(includes o175 p205)(includes o175 p210)

(waiting o176)
(includes o176 p67)(includes o176 p86)(includes o176 p160)(includes o176 p234)

(waiting o177)
(includes o177 p16)(includes o177 p133)(includes o177 p148)(includes o177 p149)(includes o177 p150)(includes o177 p183)(includes o177 p222)

(waiting o178)
(includes o178 p47)(includes o178 p64)(includes o178 p216)(includes o178 p241)

(waiting o179)
(includes o179 p7)(includes o179 p63)(includes o179 p67)(includes o179 p78)(includes o179 p129)(includes o179 p147)(includes o179 p242)

(waiting o180)
(includes o180 p45)(includes o180 p57)(includes o180 p71)(includes o180 p77)(includes o180 p78)(includes o180 p134)(includes o180 p179)(includes o180 p182)(includes o180 p183)(includes o180 p198)

(waiting o181)
(includes o181 p25)(includes o181 p53)(includes o181 p54)(includes o181 p92)(includes o181 p103)

(waiting o182)
(includes o182 p36)(includes o182 p91)(includes o182 p92)(includes o182 p155)(includes o182 p165)(includes o182 p199)(includes o182 p232)

(waiting o183)
(includes o183 p26)(includes o183 p162)(includes o183 p167)

(waiting o184)
(includes o184 p2)(includes o184 p16)(includes o184 p28)(includes o184 p44)(includes o184 p145)(includes o184 p146)(includes o184 p177)(includes o184 p187)

(waiting o185)
(includes o185 p2)(includes o185 p86)(includes o185 p87)(includes o185 p118)(includes o185 p138)(includes o185 p140)(includes o185 p161)(includes o185 p182)(includes o185 p191)(includes o185 p192)(includes o185 p229)(includes o185 p237)

(waiting o186)
(includes o186 p34)(includes o186 p69)(includes o186 p105)(includes o186 p109)(includes o186 p143)(includes o186 p150)(includes o186 p184)(includes o186 p204)

(waiting o187)
(includes o187 p36)(includes o187 p114)(includes o187 p219)(includes o187 p230)(includes o187 p238)

(waiting o188)
(includes o188 p134)(includes o188 p177)(includes o188 p189)(includes o188 p223)

(waiting o189)
(includes o189 p25)(includes o189 p36)(includes o189 p44)(includes o189 p49)(includes o189 p97)(includes o189 p107)(includes o189 p174)(includes o189 p183)(includes o189 p184)(includes o189 p222)(includes o189 p239)

(waiting o190)
(includes o190 p127)(includes o190 p157)(includes o190 p166)(includes o190 p172)(includes o190 p191)(includes o190 p221)

(waiting o191)
(includes o191 p4)(includes o191 p8)(includes o191 p62)(includes o191 p140)(includes o191 p148)(includes o191 p175)(includes o191 p224)

(waiting o192)
(includes o192 p15)(includes o192 p39)(includes o192 p64)(includes o192 p112)(includes o192 p130)(includes o192 p146)(includes o192 p168)(includes o192 p172)(includes o192 p186)

(waiting o193)
(includes o193 p102)(includes o193 p247)

(waiting o194)
(includes o194 p157)(includes o194 p200)

(waiting o195)
(includes o195 p34)(includes o195 p95)(includes o195 p149)(includes o195 p202)

(waiting o196)
(includes o196 p5)(includes o196 p9)(includes o196 p77)(includes o196 p199)(includes o196 p206)(includes o196 p229)

(waiting o197)
(includes o197 p30)(includes o197 p83)(includes o197 p100)(includes o197 p208)

(waiting o198)
(includes o198 p21)(includes o198 p45)(includes o198 p160)(includes o198 p169)(includes o198 p176)(includes o198 p188)(includes o198 p234)

(waiting o199)
(includes o199 p20)(includes o199 p66)(includes o199 p70)(includes o199 p73)(includes o199 p141)(includes o199 p142)(includes o199 p147)(includes o199 p209)(includes o199 p211)(includes o199 p218)

(waiting o200)
(includes o200 p81)(includes o200 p83)(includes o200 p118)(includes o200 p175)(includes o200 p218)(includes o200 p244)(includes o200 p246)

(waiting o201)
(includes o201 p8)(includes o201 p27)(includes o201 p97)(includes o201 p101)(includes o201 p126)(includes o201 p164)(includes o201 p173)(includes o201 p190)(includes o201 p246)

(waiting o202)
(includes o202 p40)(includes o202 p49)(includes o202 p72)(includes o202 p95)(includes o202 p168)(includes o202 p191)(includes o202 p250)

(waiting o203)
(includes o203 p79)(includes o203 p85)(includes o203 p114)(includes o203 p130)(includes o203 p136)(includes o203 p244)

(waiting o204)
(includes o204 p8)(includes o204 p34)(includes o204 p46)(includes o204 p87)(includes o204 p90)(includes o204 p145)(includes o204 p150)(includes o204 p157)(includes o204 p163)(includes o204 p187)(includes o204 p236)(includes o204 p246)

(waiting o205)
(includes o205 p4)(includes o205 p5)(includes o205 p25)(includes o205 p54)(includes o205 p111)(includes o205 p113)(includes o205 p115)

(waiting o206)
(includes o206 p51)(includes o206 p54)(includes o206 p75)(includes o206 p198)

(waiting o207)
(includes o207 p126)(includes o207 p135)(includes o207 p175)(includes o207 p245)

(waiting o208)
(includes o208 p9)(includes o208 p62)(includes o208 p86)(includes o208 p139)(includes o208 p226)

(waiting o209)
(includes o209 p94)(includes o209 p104)(includes o209 p107)(includes o209 p109)(includes o209 p120)(includes o209 p179)(includes o209 p198)(includes o209 p224)(includes o209 p236)(includes o209 p242)

(waiting o210)
(includes o210 p15)(includes o210 p136)(includes o210 p174)(includes o210 p195)

(waiting o211)
(includes o211 p33)(includes o211 p58)(includes o211 p64)(includes o211 p99)(includes o211 p105)(includes o211 p168)(includes o211 p180)(includes o211 p206)(includes o211 p214)(includes o211 p222)(includes o211 p245)

(waiting o212)
(includes o212 p75)(includes o212 p224)(includes o212 p245)

(waiting o213)
(includes o213 p28)(includes o213 p39)(includes o213 p100)(includes o213 p219)

(waiting o214)
(includes o214 p15)(includes o214 p19)(includes o214 p43)(includes o214 p74)(includes o214 p77)(includes o214 p122)(includes o214 p151)(includes o214 p153)(includes o214 p176)(includes o214 p188)(includes o214 p197)(includes o214 p227)

(waiting o215)
(includes o215 p32)(includes o215 p34)(includes o215 p150)(includes o215 p176)(includes o215 p188)(includes o215 p189)

(waiting o216)
(includes o216 p60)(includes o216 p166)

(waiting o217)
(includes o217 p32)(includes o217 p67)(includes o217 p86)(includes o217 p91)(includes o217 p93)(includes o217 p119)(includes o217 p250)

(waiting o218)
(includes o218 p25)(includes o218 p28)(includes o218 p57)(includes o218 p72)(includes o218 p78)(includes o218 p92)(includes o218 p110)(includes o218 p139)(includes o218 p161)(includes o218 p196)(includes o218 p212)(includes o218 p224)(includes o218 p229)

(waiting o219)
(includes o219 p6)(includes o219 p78)(includes o219 p92)(includes o219 p193)(includes o219 p216)(includes o219 p231)(includes o219 p237)

(waiting o220)
(includes o220 p20)(includes o220 p43)(includes o220 p45)(includes o220 p94)(includes o220 p132)

(waiting o221)
(includes o221 p99)(includes o221 p144)(includes o221 p155)(includes o221 p189)(includes o221 p227)

(waiting o222)
(includes o222 p12)(includes o222 p122)(includes o222 p125)(includes o222 p150)(includes o222 p233)(includes o222 p239)

(waiting o223)
(includes o223 p44)(includes o223 p105)(includes o223 p152)(includes o223 p156)(includes o223 p202)(includes o223 p211)

(waiting o224)
(includes o224 p20)

(waiting o225)
(includes o225 p42)(includes o225 p124)(includes o225 p171)(includes o225 p173)(includes o225 p205)

(waiting o226)
(includes o226 p2)(includes o226 p46)(includes o226 p73)(includes o226 p121)(includes o226 p148)(includes o226 p153)(includes o226 p168)(includes o226 p193)

(waiting o227)
(includes o227 p27)(includes o227 p126)(includes o227 p168)(includes o227 p217)

(waiting o228)
(includes o228 p9)(includes o228 p75)(includes o228 p108)(includes o228 p148)

(waiting o229)
(includes o229 p4)(includes o229 p41)(includes o229 p104)(includes o229 p144)(includes o229 p165)(includes o229 p201)

(waiting o230)
(includes o230 p71)(includes o230 p81)(includes o230 p90)(includes o230 p108)(includes o230 p109)(includes o230 p161)(includes o230 p193)(includes o230 p201)(includes o230 p243)(includes o230 p249)

(waiting o231)
(includes o231 p55)(includes o231 p63)(includes o231 p121)(includes o231 p146)(includes o231 p154)(includes o231 p191)(includes o231 p214)(includes o231 p222)

(waiting o232)
(includes o232 p38)(includes o232 p157)(includes o232 p212)

(waiting o233)
(includes o233 p54)(includes o233 p75)(includes o233 p192)(includes o233 p198)(includes o233 p218)

(waiting o234)
(includes o234 p5)(includes o234 p34)(includes o234 p48)(includes o234 p69)(includes o234 p76)(includes o234 p117)(includes o234 p205)(includes o234 p233)(includes o234 p241)

(waiting o235)
(includes o235 p29)(includes o235 p66)(includes o235 p80)(includes o235 p133)(includes o235 p145)(includes o235 p162)(includes o235 p167)(includes o235 p182)(includes o235 p187)(includes o235 p225)

(waiting o236)
(includes o236 p12)(includes o236 p24)(includes o236 p32)(includes o236 p37)(includes o236 p67)(includes o236 p77)(includes o236 p94)(includes o236 p119)(includes o236 p121)(includes o236 p166)(includes o236 p204)(includes o236 p205)

(waiting o237)
(includes o237 p68)(includes o237 p80)(includes o237 p89)(includes o237 p100)(includes o237 p166)(includes o237 p183)(includes o237 p250)

(waiting o238)
(includes o238 p4)(includes o238 p10)(includes o238 p36)(includes o238 p103)

(waiting o239)
(includes o239 p54)(includes o239 p149)

(waiting o240)
(includes o240 p4)(includes o240 p28)(includes o240 p49)(includes o240 p120)(includes o240 p168)(includes o240 p175)(includes o240 p212)(includes o240 p218)

(waiting o241)
(includes o241 p1)(includes o241 p37)(includes o241 p56)(includes o241 p76)(includes o241 p140)(includes o241 p200)

(waiting o242)
(includes o242 p48)(includes o242 p95)(includes o242 p107)(includes o242 p122)(includes o242 p146)(includes o242 p176)(includes o242 p242)

(waiting o243)
(includes o243 p2)(includes o243 p42)(includes o243 p160)(includes o243 p240)

(waiting o244)
(includes o244 p35)(includes o244 p36)(includes o244 p129)(includes o244 p230)(includes o244 p245)(includes o244 p249)

(waiting o245)
(includes o245 p28)(includes o245 p73)(includes o245 p146)(includes o245 p179)(includes o245 p184)(includes o245 p207)(includes o245 p217)(includes o245 p239)

(waiting o246)
(includes o246 p18)(includes o246 p45)(includes o246 p100)(includes o246 p142)(includes o246 p191)(includes o246 p195)(includes o246 p247)

(waiting o247)
(includes o247 p44)(includes o247 p78)(includes o247 p89)(includes o247 p108)(includes o247 p117)(includes o247 p167)

(waiting o248)
(includes o248 p6)(includes o248 p8)(includes o248 p28)(includes o248 p49)(includes o248 p53)(includes o248 p68)(includes o248 p82)(includes o248 p109)(includes o248 p135)(includes o248 p150)(includes o248 p167)

(waiting o249)
(includes o249 p30)(includes o249 p96)(includes o249 p186)(includes o249 p228)(includes o249 p244)(includes o249 p245)

(waiting o250)
(includes o250 p13)(includes o250 p174)(includes o250 p196)(includes o250 p208)(includes o250 p226)

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

