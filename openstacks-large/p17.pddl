(define (problem os-sequencedstrips-p270_2)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210 n211 n212 n213 n214 n215 n216 n217 n218 n219 n220 n221 n222 n223 n224 n225 n226 n227 n228 n229 n230 n231 n232 n233 n234 n235 n236 n237 n238 n239 n240 n241 n242 n243 n244 n245 n246 n247 n248 n249 n250 n251 n252 n253 n254 n255 n256 n257 n258 n259 n260 n261 n262 n263 n264 n265 n266 n267 n268 n269 n270  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) (next-count n210 n211) (next-count n211 n212) (next-count n212 n213) (next-count n213 n214) (next-count n214 n215) (next-count n215 n216) (next-count n216 n217) (next-count n217 n218) (next-count n218 n219) (next-count n219 n220) (next-count n220 n221) (next-count n221 n222) (next-count n222 n223) (next-count n223 n224) (next-count n224 n225) (next-count n225 n226) (next-count n226 n227) (next-count n227 n228) (next-count n228 n229) (next-count n229 n230) (next-count n230 n231) (next-count n231 n232) (next-count n232 n233) (next-count n233 n234) (next-count n234 n235) (next-count n235 n236) (next-count n236 n237) (next-count n237 n238) (next-count n238 n239) (next-count n239 n240) (next-count n240 n241) (next-count n241 n242) (next-count n242 n243) (next-count n243 n244) (next-count n244 n245) (next-count n245 n246) (next-count n246 n247) (next-count n247 n248) (next-count n248 n249) (next-count n249 n250) (next-count n250 n251) (next-count n251 n252) (next-count n252 n253) (next-count n253 n254) (next-count n254 n255) (next-count n255 n256) (next-count n256 n257) (next-count n257 n258) (next-count n258 n259) (next-count n259 n260) (next-count n260 n261) (next-count n261 n262) (next-count n262 n263) (next-count n263 n264) (next-count n264 n265) (next-count n265 n266) (next-count n266 n267) (next-count n267 n268) (next-count n268 n269) (next-count n269 n270) 
(stacks-avail n0)

(waiting o1)
(includes o1 p2)(includes o1 p16)(includes o1 p28)(includes o1 p92)(includes o1 p171)(includes o1 p218)(includes o1 p247)

(waiting o2)
(includes o2 p10)(includes o2 p47)(includes o2 p59)(includes o2 p74)(includes o2 p157)(includes o2 p180)(includes o2 p212)(includes o2 p253)

(waiting o3)
(includes o3 p90)(includes o3 p175)(includes o3 p232)

(waiting o4)
(includes o4 p37)(includes o4 p78)(includes o4 p241)

(waiting o5)
(includes o5 p6)(includes o5 p69)(includes o5 p147)(includes o5 p150)(includes o5 p166)(includes o5 p190)

(waiting o6)
(includes o6 p47)(includes o6 p99)(includes o6 p112)(includes o6 p165)(includes o6 p197)(includes o6 p213)(includes o6 p222)(includes o6 p255)(includes o6 p266)

(waiting o7)
(includes o7 p119)(includes o7 p140)(includes o7 p159)

(waiting o8)
(includes o8 p46)(includes o8 p107)(includes o8 p185)(includes o8 p187)(includes o8 p223)

(waiting o9)
(includes o9 p32)(includes o9 p49)(includes o9 p83)(includes o9 p161)(includes o9 p171)(includes o9 p198)(includes o9 p262)

(waiting o10)
(includes o10 p10)(includes o10 p39)(includes o10 p48)(includes o10 p53)(includes o10 p101)(includes o10 p131)

(waiting o11)
(includes o11 p91)(includes o11 p150)(includes o11 p151)(includes o11 p173)(includes o11 p212)

(waiting o12)
(includes o12 p17)(includes o12 p29)(includes o12 p74)(includes o12 p80)(includes o12 p124)(includes o12 p156)(includes o12 p164)(includes o12 p193)(includes o12 p216)

(waiting o13)
(includes o13 p26)(includes o13 p78)(includes o13 p207)(includes o13 p211)(includes o13 p260)

(waiting o14)
(includes o14 p16)(includes o14 p18)(includes o14 p158)(includes o14 p191)(includes o14 p232)(includes o14 p240)

(waiting o15)
(includes o15 p66)(includes o15 p163)(includes o15 p167)(includes o15 p216)

(waiting o16)
(includes o16 p22)(includes o16 p66)(includes o16 p70)(includes o16 p153)(includes o16 p158)(includes o16 p161)(includes o16 p180)(includes o16 p231)(includes o16 p236)

(waiting o17)
(includes o17 p24)(includes o17 p63)(includes o17 p85)(includes o17 p122)(includes o17 p241)(includes o17 p250)

(waiting o18)
(includes o18 p68)(includes o18 p132)(includes o18 p144)(includes o18 p188)(includes o18 p202)(includes o18 p219)(includes o18 p267)

(waiting o19)
(includes o19 p1)(includes o19 p14)(includes o19 p43)(includes o19 p45)(includes o19 p48)(includes o19 p131)(includes o19 p138)(includes o19 p142)(includes o19 p163)(includes o19 p172)

(waiting o20)
(includes o20 p9)(includes o20 p10)(includes o20 p13)(includes o20 p15)(includes o20 p48)(includes o20 p144)(includes o20 p153)(includes o20 p157)(includes o20 p195)(includes o20 p206)

(waiting o21)
(includes o21 p26)(includes o21 p54)(includes o21 p75)(includes o21 p239)(includes o21 p248)

(waiting o22)
(includes o22 p38)(includes o22 p93)(includes o22 p113)(includes o22 p131)(includes o22 p170)(includes o22 p239)(includes o22 p250)

(waiting o23)
(includes o23 p55)(includes o23 p194)(includes o23 p238)

(waiting o24)
(includes o24 p1)(includes o24 p40)(includes o24 p84)(includes o24 p108)(includes o24 p128)(includes o24 p174)(includes o24 p190)

(waiting o25)
(includes o25 p25)(includes o25 p27)(includes o25 p118)(includes o25 p125)(includes o25 p128)(includes o25 p152)(includes o25 p246)

(waiting o26)
(includes o26 p34)(includes o26 p75)(includes o26 p182)(includes o26 p196)

(waiting o27)
(includes o27 p43)(includes o27 p51)(includes o27 p94)(includes o27 p106)(includes o27 p198)(includes o27 p244)

(waiting o28)
(includes o28 p34)(includes o28 p75)(includes o28 p151)(includes o28 p180)(includes o28 p221)(includes o28 p227)(includes o28 p238)(includes o28 p254)

(waiting o29)
(includes o29 p22)(includes o29 p28)(includes o29 p125)(includes o29 p203)(includes o29 p238)

(waiting o30)
(includes o30 p2)(includes o30 p55)(includes o30 p58)(includes o30 p174)(includes o30 p230)(includes o30 p258)(includes o30 p261)

(waiting o31)
(includes o31 p26)(includes o31 p27)(includes o31 p98)(includes o31 p120)(includes o31 p123)(includes o31 p200)

(waiting o32)
(includes o32 p88)(includes o32 p146)(includes o32 p160)(includes o32 p176)(includes o32 p213)

(waiting o33)
(includes o33 p7)(includes o33 p65)(includes o33 p72)(includes o33 p198)(includes o33 p222)(includes o33 p233)(includes o33 p260)

(waiting o34)
(includes o34 p21)(includes o34 p47)(includes o34 p69)(includes o34 p127)(includes o34 p149)(includes o34 p169)(includes o34 p171)(includes o34 p178)(includes o34 p206)(includes o34 p219)

(waiting o35)
(includes o35 p95)(includes o35 p125)(includes o35 p133)(includes o35 p169)(includes o35 p176)

(waiting o36)
(includes o36 p62)(includes o36 p228)

(waiting o37)
(includes o37 p50)(includes o37 p54)(includes o37 p89)(includes o37 p112)(includes o37 p126)(includes o37 p132)(includes o37 p160)(includes o37 p207)

(waiting o38)
(includes o38 p43)(includes o38 p50)(includes o38 p118)(includes o38 p147)(includes o38 p149)(includes o38 p157)(includes o38 p203)(includes o38 p224)(includes o38 p257)

(waiting o39)
(includes o39 p16)(includes o39 p71)(includes o39 p94)(includes o39 p217)(includes o39 p226)(includes o39 p234)(includes o39 p258)

(waiting o40)
(includes o40 p59)(includes o40 p62)(includes o40 p94)(includes o40 p100)(includes o40 p166)(includes o40 p182)(includes o40 p231)

(waiting o41)
(includes o41 p88)(includes o41 p92)(includes o41 p99)(includes o41 p188)(includes o41 p258)

(waiting o42)
(includes o42 p7)(includes o42 p54)(includes o42 p93)(includes o42 p109)(includes o42 p127)(includes o42 p132)(includes o42 p199)(includes o42 p224)

(waiting o43)
(includes o43 p8)(includes o43 p27)(includes o43 p32)(includes o43 p42)(includes o43 p44)(includes o43 p50)(includes o43 p111)(includes o43 p115)(includes o43 p241)

(waiting o44)
(includes o44 p23)(includes o44 p43)(includes o44 p53)(includes o44 p131)(includes o44 p163)(includes o44 p199)

(waiting o45)
(includes o45 p15)(includes o45 p17)(includes o45 p19)(includes o45 p39)(includes o45 p75)(includes o45 p76)(includes o45 p105)(includes o45 p118)(includes o45 p195)(includes o45 p246)(includes o45 p249)

(waiting o46)
(includes o46 p113)(includes o46 p244)

(waiting o47)
(includes o47 p3)(includes o47 p11)(includes o47 p112)(includes o47 p160)(includes o47 p176)(includes o47 p212)(includes o47 p249)

(waiting o48)
(includes o48 p8)(includes o48 p11)(includes o48 p18)(includes o48 p42)(includes o48 p71)(includes o48 p168)(includes o48 p185)(includes o48 p198)(includes o48 p213)

(waiting o49)
(includes o49 p100)(includes o49 p104)(includes o49 p129)(includes o49 p142)(includes o49 p157)(includes o49 p224)(includes o49 p246)(includes o49 p254)

(waiting o50)
(includes o50 p18)(includes o50 p35)(includes o50 p100)(includes o50 p125)

(waiting o51)
(includes o51 p98)(includes o51 p111)(includes o51 p161)(includes o51 p182)(includes o51 p190)(includes o51 p204)(includes o51 p242)

(waiting o52)
(includes o52 p13)(includes o52 p16)(includes o52 p60)(includes o52 p78)(includes o52 p86)(includes o52 p91)(includes o52 p108)(includes o52 p116)(includes o52 p154)(includes o52 p202)(includes o52 p207)(includes o52 p270)

(waiting o53)
(includes o53 p13)(includes o53 p14)(includes o53 p203)(includes o53 p220)(includes o53 p249)

(waiting o54)
(includes o54 p70)(includes o54 p110)(includes o54 p117)(includes o54 p138)(includes o54 p139)(includes o54 p183)(includes o54 p263)

(waiting o55)
(includes o55 p49)(includes o55 p207)(includes o55 p250)

(waiting o56)
(includes o56 p38)(includes o56 p76)(includes o56 p95)(includes o56 p225)

(waiting o57)
(includes o57 p124)(includes o57 p183)(includes o57 p244)(includes o57 p261)(includes o57 p264)

(waiting o58)
(includes o58 p75)(includes o58 p84)(includes o58 p92)(includes o58 p102)(includes o58 p123)(includes o58 p186)(includes o58 p227)

(waiting o59)
(includes o59 p63)(includes o59 p79)(includes o59 p127)(includes o59 p144)(includes o59 p157)(includes o59 p253)

(waiting o60)
(includes o60 p198)(includes o60 p203)(includes o60 p204)(includes o60 p220)

(waiting o61)
(includes o61 p14)(includes o61 p57)(includes o61 p110)(includes o61 p196)(includes o61 p206)(includes o61 p232)(includes o61 p239)(includes o61 p258)(includes o61 p264)

(waiting o62)
(includes o62 p65)(includes o62 p86)(includes o62 p152)(includes o62 p166)

(waiting o63)
(includes o63 p77)(includes o63 p115)(includes o63 p119)(includes o63 p126)(includes o63 p195)(includes o63 p201)(includes o63 p250)(includes o63 p262)

(waiting o64)
(includes o64 p115)(includes o64 p118)(includes o64 p127)(includes o64 p157)(includes o64 p196)(includes o64 p204)(includes o64 p213)(includes o64 p254)

(waiting o65)
(includes o65 p1)(includes o65 p106)(includes o65 p137)(includes o65 p187)(includes o65 p218)(includes o65 p221)

(waiting o66)
(includes o66 p48)(includes o66 p108)(includes o66 p127)(includes o66 p133)(includes o66 p143)(includes o66 p154)(includes o66 p181)(includes o66 p219)(includes o66 p259)

(waiting o67)
(includes o67 p75)(includes o67 p204)(includes o67 p214)(includes o67 p252)(includes o67 p265)(includes o67 p267)

(waiting o68)
(includes o68 p16)(includes o68 p19)(includes o68 p70)(includes o68 p114)(includes o68 p183)

(waiting o69)
(includes o69 p30)(includes o69 p34)(includes o69 p121)(includes o69 p131)(includes o69 p141)(includes o69 p143)(includes o69 p173)(includes o69 p252)(includes o69 p263)

(waiting o70)
(includes o70 p36)(includes o70 p257)(includes o70 p259)

(waiting o71)
(includes o71 p40)(includes o71 p47)(includes o71 p102)(includes o71 p104)

(waiting o72)
(includes o72 p95)(includes o72 p118)(includes o72 p199)(includes o72 p267)(includes o72 p268)(includes o72 p270)

(waiting o73)
(includes o73 p3)(includes o73 p86)(includes o73 p109)(includes o73 p129)(includes o73 p163)(includes o73 p175)(includes o73 p180)

(waiting o74)
(includes o74 p53)(includes o74 p177)(includes o74 p192)(includes o74 p193)(includes o74 p217)

(waiting o75)
(includes o75 p87)(includes o75 p133)(includes o75 p167)(includes o75 p179)(includes o75 p209)(includes o75 p232)(includes o75 p234)(includes o75 p241)

(waiting o76)
(includes o76 p45)(includes o76 p63)(includes o76 p64)(includes o76 p91)(includes o76 p99)(includes o76 p164)(includes o76 p188)(includes o76 p240)

(waiting o77)
(includes o77 p40)(includes o77 p43)(includes o77 p101)(includes o77 p151)(includes o77 p156)(includes o77 p161)(includes o77 p174)

(waiting o78)
(includes o78 p45)(includes o78 p69)(includes o78 p97)(includes o78 p106)(includes o78 p209)

(waiting o79)
(includes o79 p7)(includes o79 p15)(includes o79 p50)(includes o79 p100)(includes o79 p156)(includes o79 p175)(includes o79 p216)(includes o79 p221)(includes o79 p226)(includes o79 p228)(includes o79 p241)(includes o79 p255)(includes o79 p264)(includes o79 p267)

(waiting o80)
(includes o80 p60)(includes o80 p133)(includes o80 p186)(includes o80 p225)(includes o80 p226)(includes o80 p259)

(waiting o81)
(includes o81 p56)(includes o81 p59)(includes o81 p68)(includes o81 p72)(includes o81 p107)(includes o81 p145)(includes o81 p174)(includes o81 p225)(includes o81 p263)

(waiting o82)
(includes o82 p10)(includes o82 p63)(includes o82 p100)(includes o82 p101)(includes o82 p140)(includes o82 p192)(includes o82 p212)(includes o82 p219)(includes o82 p229)

(waiting o83)
(includes o83 p85)(includes o83 p192)(includes o83 p204)(includes o83 p212)

(waiting o84)
(includes o84 p14)(includes o84 p50)(includes o84 p62)(includes o84 p68)(includes o84 p71)(includes o84 p108)(includes o84 p123)(includes o84 p155)(includes o84 p203)(includes o84 p221)(includes o84 p229)

(waiting o85)
(includes o85 p63)(includes o85 p160)(includes o85 p223)(includes o85 p240)

(waiting o86)
(includes o86 p22)(includes o86 p83)(includes o86 p130)(includes o86 p144)(includes o86 p153)(includes o86 p158)(includes o86 p203)(includes o86 p238)(includes o86 p264)

(waiting o87)
(includes o87 p16)(includes o87 p96)(includes o87 p173)(includes o87 p231)

(waiting o88)
(includes o88 p42)(includes o88 p46)(includes o88 p81)(includes o88 p152)(includes o88 p180)

(waiting o89)
(includes o89 p32)(includes o89 p67)(includes o89 p107)(includes o89 p115)(includes o89 p151)(includes o89 p200)(includes o89 p230)(includes o89 p233)(includes o89 p244)(includes o89 p255)(includes o89 p268)

(waiting o90)
(includes o90 p29)(includes o90 p67)(includes o90 p233)(includes o90 p265)

(waiting o91)
(includes o91 p58)(includes o91 p71)(includes o91 p74)(includes o91 p115)(includes o91 p200)(includes o91 p225)(includes o91 p237)(includes o91 p252)

(waiting o92)
(includes o92 p7)(includes o92 p51)(includes o92 p118)(includes o92 p140)(includes o92 p161)(includes o92 p180)(includes o92 p234)(includes o92 p248)(includes o92 p251)

(waiting o93)
(includes o93 p2)(includes o93 p8)(includes o93 p17)(includes o93 p62)(includes o93 p72)(includes o93 p111)(includes o93 p120)(includes o93 p168)(includes o93 p187)(includes o93 p209)(includes o93 p235)(includes o93 p244)(includes o93 p267)

(waiting o94)
(includes o94 p144)(includes o94 p173)

(waiting o95)
(includes o95 p2)(includes o95 p23)(includes o95 p62)(includes o95 p108)(includes o95 p145)(includes o95 p159)(includes o95 p184)(includes o95 p190)

(waiting o96)
(includes o96 p27)(includes o96 p72)(includes o96 p113)(includes o96 p227)(includes o96 p235)(includes o96 p244)(includes o96 p266)

(waiting o97)
(includes o97 p20)(includes o97 p47)(includes o97 p50)(includes o97 p137)(includes o97 p269)

(waiting o98)
(includes o98 p8)(includes o98 p34)(includes o98 p47)(includes o98 p166)(includes o98 p231)

(waiting o99)
(includes o99 p55)(includes o99 p72)(includes o99 p90)(includes o99 p191)(includes o99 p228)(includes o99 p233)(includes o99 p255)

(waiting o100)
(includes o100 p18)(includes o100 p33)(includes o100 p50)(includes o100 p62)(includes o100 p99)(includes o100 p169)(includes o100 p208)(includes o100 p222)

(waiting o101)
(includes o101 p20)(includes o101 p88)(includes o101 p121)(includes o101 p139)(includes o101 p167)(includes o101 p239)

(waiting o102)
(includes o102 p19)(includes o102 p29)(includes o102 p42)(includes o102 p98)(includes o102 p154)(includes o102 p180)(includes o102 p207)

(waiting o103)
(includes o103 p112)(includes o103 p129)(includes o103 p179)

(waiting o104)
(includes o104 p41)(includes o104 p51)(includes o104 p59)(includes o104 p61)(includes o104 p95)(includes o104 p120)(includes o104 p161)(includes o104 p261)

(waiting o105)
(includes o105 p4)(includes o105 p17)(includes o105 p36)(includes o105 p127)(includes o105 p213)(includes o105 p263)

(waiting o106)
(includes o106 p53)(includes o106 p115)(includes o106 p132)(includes o106 p197)(includes o106 p263)

(waiting o107)
(includes o107 p65)(includes o107 p71)(includes o107 p72)(includes o107 p80)(includes o107 p105)(includes o107 p203)(includes o107 p235)

(waiting o108)
(includes o108 p9)(includes o108 p23)(includes o108 p45)(includes o108 p50)(includes o108 p72)(includes o108 p106)(includes o108 p168)(includes o108 p183)(includes o108 p233)

(waiting o109)
(includes o109 p99)(includes o109 p106)(includes o109 p128)(includes o109 p165)(includes o109 p210)(includes o109 p227)(includes o109 p254)

(waiting o110)
(includes o110 p30)(includes o110 p70)(includes o110 p125)

(waiting o111)
(includes o111 p31)(includes o111 p68)(includes o111 p117)(includes o111 p133)

(waiting o112)
(includes o112 p1)(includes o112 p88)(includes o112 p103)(includes o112 p120)(includes o112 p140)(includes o112 p165)(includes o112 p178)(includes o112 p197)(includes o112 p218)(includes o112 p255)

(waiting o113)
(includes o113 p53)(includes o113 p114)(includes o113 p122)(includes o113 p133)(includes o113 p173)(includes o113 p190)(includes o113 p267)

(waiting o114)
(includes o114 p34)(includes o114 p139)(includes o114 p175)(includes o114 p267)

(waiting o115)
(includes o115 p25)(includes o115 p169)(includes o115 p185)(includes o115 p231)(includes o115 p267)

(waiting o116)
(includes o116 p91)(includes o116 p108)(includes o116 p245)

(waiting o117)
(includes o117 p14)(includes o117 p66)(includes o117 p87)(includes o117 p199)

(waiting o118)
(includes o118 p33)(includes o118 p104)(includes o118 p173)(includes o118 p243)

(waiting o119)
(includes o119 p34)(includes o119 p190)(includes o119 p211)(includes o119 p224)(includes o119 p243)(includes o119 p267)

(waiting o120)
(includes o120 p23)(includes o120 p27)(includes o120 p46)(includes o120 p112)

(waiting o121)
(includes o121 p22)(includes o121 p25)(includes o121 p77)(includes o121 p101)(includes o121 p130)(includes o121 p149)(includes o121 p161)(includes o121 p242)(includes o121 p258)

(waiting o122)
(includes o122 p51)(includes o122 p56)(includes o122 p74)(includes o122 p150)(includes o122 p270)

(waiting o123)
(includes o123 p31)(includes o123 p140)(includes o123 p215)(includes o123 p244)(includes o123 p247)

(waiting o124)
(includes o124 p36)(includes o124 p56)(includes o124 p62)(includes o124 p63)(includes o124 p69)(includes o124 p76)(includes o124 p131)(includes o124 p137)(includes o124 p232)(includes o124 p267)

(waiting o125)
(includes o125 p5)(includes o125 p11)(includes o125 p45)(includes o125 p109)(includes o125 p126)(includes o125 p230)(includes o125 p236)(includes o125 p256)

(waiting o126)
(includes o126 p85)(includes o126 p182)(includes o126 p235)(includes o126 p241)

(waiting o127)
(includes o127 p50)(includes o127 p68)(includes o127 p243)

(waiting o128)
(includes o128 p113)(includes o128 p137)(includes o128 p178)(includes o128 p235)

(waiting o129)
(includes o129 p4)(includes o129 p6)(includes o129 p49)(includes o129 p64)(includes o129 p66)(includes o129 p168)(includes o129 p238)(includes o129 p244)

(waiting o130)
(includes o130 p63)(includes o130 p197)(includes o130 p233)

(waiting o131)
(includes o131 p42)(includes o131 p45)(includes o131 p68)(includes o131 p81)(includes o131 p109)(includes o131 p152)(includes o131 p212)(includes o131 p229)(includes o131 p259)

(waiting o132)
(includes o132 p47)(includes o132 p108)(includes o132 p201)

(waiting o133)
(includes o133 p11)(includes o133 p35)(includes o133 p215)

(waiting o134)
(includes o134 p28)(includes o134 p63)(includes o134 p174)

(waiting o135)
(includes o135 p6)(includes o135 p40)(includes o135 p157)(includes o135 p189)(includes o135 p237)

(waiting o136)
(includes o136 p2)(includes o136 p50)(includes o136 p119)(includes o136 p143)(includes o136 p270)

(waiting o137)
(includes o137 p1)(includes o137 p9)(includes o137 p151)(includes o137 p170)(includes o137 p180)(includes o137 p185)(includes o137 p213)(includes o137 p260)(includes o137 p265)

(waiting o138)
(includes o138 p48)(includes o138 p114)(includes o138 p236)

(waiting o139)
(includes o139 p17)(includes o139 p18)(includes o139 p30)(includes o139 p78)(includes o139 p89)(includes o139 p225)(includes o139 p254)

(waiting o140)
(includes o140 p123)(includes o140 p191)(includes o140 p204)(includes o140 p232)

(waiting o141)
(includes o141 p17)(includes o141 p38)(includes o141 p61)(includes o141 p104)(includes o141 p121)(includes o141 p214)

(waiting o142)
(includes o142 p84)(includes o142 p98)(includes o142 p244)

(waiting o143)
(includes o143 p36)(includes o143 p94)(includes o143 p167)

(waiting o144)
(includes o144 p52)(includes o144 p79)(includes o144 p140)(includes o144 p189)(includes o144 p210)

(waiting o145)
(includes o145 p19)(includes o145 p42)(includes o145 p61)(includes o145 p72)(includes o145 p109)(includes o145 p171)

(waiting o146)
(includes o146 p17)(includes o146 p80)(includes o146 p102)(includes o146 p109)(includes o146 p112)(includes o146 p187)(includes o146 p232)(includes o146 p253)

(waiting o147)
(includes o147 p6)(includes o147 p19)(includes o147 p45)(includes o147 p70)(includes o147 p179)(includes o147 p190)(includes o147 p260)

(waiting o148)
(includes o148 p74)(includes o148 p83)(includes o148 p97)(includes o148 p112)(includes o148 p138)(includes o148 p190)(includes o148 p207)(includes o148 p219)

(waiting o149)
(includes o149 p86)(includes o149 p90)(includes o149 p151)(includes o149 p166)(includes o149 p237)

(waiting o150)
(includes o150 p35)(includes o150 p85)(includes o150 p127)(includes o150 p138)(includes o150 p173)(includes o150 p215)(includes o150 p237)(includes o150 p261)

(waiting o151)
(includes o151 p121)(includes o151 p162)(includes o151 p171)

(waiting o152)
(includes o152 p30)(includes o152 p85)(includes o152 p160)(includes o152 p181)(includes o152 p262)

(waiting o153)
(includes o153 p9)(includes o153 p77)(includes o153 p93)(includes o153 p135)(includes o153 p162)(includes o153 p187)(includes o153 p194)(includes o153 p203)(includes o153 p245)

(waiting o154)
(includes o154 p9)(includes o154 p20)(includes o154 p32)(includes o154 p66)(includes o154 p100)(includes o154 p133)(includes o154 p149)(includes o154 p164)(includes o154 p196)(includes o154 p250)(includes o154 p252)

(waiting o155)
(includes o155 p5)(includes o155 p197)

(waiting o156)
(includes o156 p41)(includes o156 p81)(includes o156 p161)(includes o156 p169)(includes o156 p180)(includes o156 p181)(includes o156 p200)

(waiting o157)
(includes o157 p19)(includes o157 p106)(includes o157 p127)(includes o157 p209)(includes o157 p249)(includes o157 p250)(includes o157 p259)

(waiting o158)
(includes o158 p30)(includes o158 p67)(includes o158 p73)(includes o158 p86)(includes o158 p88)(includes o158 p143)(includes o158 p161)(includes o158 p202)(includes o158 p263)

(waiting o159)
(includes o159 p11)(includes o159 p29)(includes o159 p40)(includes o159 p46)(includes o159 p50)(includes o159 p100)(includes o159 p178)(includes o159 p217)(includes o159 p234)(includes o159 p266)

(waiting o160)
(includes o160 p30)(includes o160 p56)(includes o160 p67)(includes o160 p126)(includes o160 p254)

(waiting o161)
(includes o161 p40)(includes o161 p79)(includes o161 p86)(includes o161 p161)(includes o161 p175)(includes o161 p238)(includes o161 p247)(includes o161 p259)(includes o161 p260)

(waiting o162)
(includes o162 p35)(includes o162 p49)(includes o162 p135)(includes o162 p150)(includes o162 p169)

(waiting o163)
(includes o163 p102)(includes o163 p106)(includes o163 p140)

(waiting o164)
(includes o164 p81)(includes o164 p86)(includes o164 p88)(includes o164 p132)(includes o164 p214)(includes o164 p234)(includes o164 p254)(includes o164 p261)

(waiting o165)
(includes o165 p68)(includes o165 p82)(includes o165 p130)(includes o165 p182)(includes o165 p204)(includes o165 p244)

(waiting o166)
(includes o166 p139)(includes o166 p188)(includes o166 p219)(includes o166 p227)

(waiting o167)
(includes o167 p3)(includes o167 p81)(includes o167 p107)(includes o167 p136)(includes o167 p150)(includes o167 p180)(includes o167 p257)

(waiting o168)
(includes o168 p2)(includes o168 p21)(includes o168 p49)(includes o168 p70)(includes o168 p138)(includes o168 p172)(includes o168 p210)(includes o168 p237)

(waiting o169)
(includes o169 p67)(includes o169 p108)(includes o169 p164)(includes o169 p255)

(waiting o170)
(includes o170 p22)(includes o170 p50)(includes o170 p63)(includes o170 p258)

(waiting o171)
(includes o171 p35)(includes o171 p63)(includes o171 p67)(includes o171 p68)(includes o171 p86)(includes o171 p136)(includes o171 p202)(includes o171 p208)

(waiting o172)
(includes o172 p29)(includes o172 p37)(includes o172 p46)(includes o172 p105)(includes o172 p111)(includes o172 p133)(includes o172 p148)(includes o172 p187)(includes o172 p209)(includes o172 p213)(includes o172 p216)(includes o172 p225)(includes o172 p229)(includes o172 p232)

(waiting o173)
(includes o173 p33)(includes o173 p62)(includes o173 p78)(includes o173 p140)(includes o173 p175)(includes o173 p224)

(waiting o174)
(includes o174 p21)(includes o174 p88)(includes o174 p168)(includes o174 p233)(includes o174 p234)(includes o174 p249)

(waiting o175)
(includes o175 p49)(includes o175 p87)(includes o175 p191)(includes o175 p225)(includes o175 p227)(includes o175 p238)(includes o175 p263)(includes o175 p264)

(waiting o176)
(includes o176 p20)(includes o176 p48)(includes o176 p63)(includes o176 p86)(includes o176 p154)(includes o176 p164)(includes o176 p223)(includes o176 p264)

(waiting o177)
(includes o177 p40)(includes o177 p213)(includes o177 p268)

(waiting o178)
(includes o178 p8)(includes o178 p70)(includes o178 p118)(includes o178 p163)(includes o178 p168)(includes o178 p198)

(waiting o179)
(includes o179 p42)(includes o179 p105)(includes o179 p111)(includes o179 p122)(includes o179 p124)(includes o179 p147)(includes o179 p153)(includes o179 p163)(includes o179 p195)(includes o179 p200)(includes o179 p203)(includes o179 p224)(includes o179 p239)(includes o179 p253)

(waiting o180)
(includes o180 p15)(includes o180 p112)(includes o180 p180)(includes o180 p192)(includes o180 p234)

(waiting o181)
(includes o181 p7)(includes o181 p78)(includes o181 p83)(includes o181 p232)(includes o181 p238)(includes o181 p258)

(waiting o182)
(includes o182 p59)(includes o182 p183)(includes o182 p242)(includes o182 p251)

(waiting o183)
(includes o183 p113)(includes o183 p142)(includes o183 p239)

(waiting o184)
(includes o184 p197)(includes o184 p205)(includes o184 p206)

(waiting o185)
(includes o185 p16)(includes o185 p49)(includes o185 p87)(includes o185 p124)(includes o185 p128)(includes o185 p130)(includes o185 p168)(includes o185 p212)(includes o185 p235)

(waiting o186)
(includes o186 p22)(includes o186 p51)(includes o186 p58)(includes o186 p84)(includes o186 p171)(includes o186 p186)(includes o186 p216)(includes o186 p224)

(waiting o187)
(includes o187 p5)(includes o187 p107)(includes o187 p114)(includes o187 p138)(includes o187 p181)(includes o187 p220)

(waiting o188)
(includes o188 p23)(includes o188 p169)(includes o188 p205)(includes o188 p232)(includes o188 p243)

(waiting o189)
(includes o189 p48)(includes o189 p69)(includes o189 p79)(includes o189 p96)(includes o189 p128)(includes o189 p186)(includes o189 p229)(includes o189 p233)(includes o189 p252)(includes o189 p261)

(waiting o190)
(includes o190 p56)(includes o190 p103)(includes o190 p109)(includes o190 p171)(includes o190 p207)(includes o190 p243)

(waiting o191)
(includes o191 p68)(includes o191 p80)(includes o191 p121)(includes o191 p159)

(waiting o192)
(includes o192 p12)(includes o192 p17)(includes o192 p49)(includes o192 p53)(includes o192 p142)(includes o192 p147)(includes o192 p163)(includes o192 p167)(includes o192 p184)(includes o192 p187)(includes o192 p250)(includes o192 p252)

(waiting o193)
(includes o193 p70)(includes o193 p85)(includes o193 p155)(includes o193 p207)

(waiting o194)
(includes o194 p15)(includes o194 p37)(includes o194 p106)(includes o194 p199)(includes o194 p200)

(waiting o195)
(includes o195 p73)(includes o195 p87)(includes o195 p95)(includes o195 p125)(includes o195 p159)(includes o195 p173)(includes o195 p194)(includes o195 p238)(includes o195 p254)

(waiting o196)
(includes o196 p3)(includes o196 p12)(includes o196 p27)(includes o196 p56)(includes o196 p151)(includes o196 p158)(includes o196 p204)(includes o196 p232)

(waiting o197)
(includes o197 p65)(includes o197 p95)(includes o197 p124)(includes o197 p131)(includes o197 p157)(includes o197 p193)(includes o197 p203)(includes o197 p217)(includes o197 p224)(includes o197 p252)(includes o197 p260)(includes o197 p265)

(waiting o198)
(includes o198 p39)(includes o198 p114)(includes o198 p138)(includes o198 p144)(includes o198 p203)

(waiting o199)
(includes o199 p31)(includes o199 p41)(includes o199 p54)(includes o199 p107)(includes o199 p203)(includes o199 p233)(includes o199 p256)

(waiting o200)
(includes o200 p3)(includes o200 p93)(includes o200 p112)(includes o200 p234)(includes o200 p252)

(waiting o201)
(includes o201 p44)(includes o201 p87)(includes o201 p123)(includes o201 p132)(includes o201 p234)(includes o201 p240)(includes o201 p247)(includes o201 p259)

(waiting o202)
(includes o202 p81)(includes o202 p106)(includes o202 p123)(includes o202 p168)(includes o202 p242)(includes o202 p267)

(waiting o203)
(includes o203 p30)(includes o203 p107)(includes o203 p145)(includes o203 p184)(includes o203 p186)(includes o203 p193)(includes o203 p237)

(waiting o204)
(includes o204 p22)(includes o204 p32)(includes o204 p99)(includes o204 p145)

(waiting o205)
(includes o205 p9)(includes o205 p26)(includes o205 p37)(includes o205 p51)(includes o205 p52)(includes o205 p142)(includes o205 p171)(includes o205 p192)

(waiting o206)
(includes o206 p62)(includes o206 p77)(includes o206 p79)(includes o206 p184)(includes o206 p195)

(waiting o207)
(includes o207 p45)(includes o207 p101)

(waiting o208)
(includes o208 p131)(includes o208 p224)(includes o208 p244)

(waiting o209)
(includes o209 p12)(includes o209 p63)(includes o209 p88)(includes o209 p100)(includes o209 p105)(includes o209 p111)(includes o209 p120)(includes o209 p134)(includes o209 p212)(includes o209 p239)(includes o209 p252)(includes o209 p257)(includes o209 p266)

(waiting o210)
(includes o210 p10)(includes o210 p42)(includes o210 p45)(includes o210 p92)(includes o210 p158)(includes o210 p209)(includes o210 p216)(includes o210 p268)

(waiting o211)
(includes o211 p20)(includes o211 p35)(includes o211 p49)(includes o211 p92)(includes o211 p165)(includes o211 p177)

(waiting o212)
(includes o212 p43)(includes o212 p73)(includes o212 p104)(includes o212 p143)(includes o212 p177)(includes o212 p218)(includes o212 p223)(includes o212 p238)(includes o212 p248)(includes o212 p261)

(waiting o213)
(includes o213 p69)(includes o213 p122)(includes o213 p185)(includes o213 p207)(includes o213 p239)

(waiting o214)
(includes o214 p40)(includes o214 p71)(includes o214 p172)(includes o214 p248)

(waiting o215)
(includes o215 p24)(includes o215 p30)(includes o215 p47)(includes o215 p72)(includes o215 p78)(includes o215 p242)(includes o215 p247)(includes o215 p259)

(waiting o216)
(includes o216 p60)(includes o216 p126)(includes o216 p160)(includes o216 p197)(includes o216 p230)(includes o216 p251)

(waiting o217)
(includes o217 p3)(includes o217 p105)

(waiting o218)
(includes o218 p24)(includes o218 p36)(includes o218 p65)(includes o218 p85)(includes o218 p115)(includes o218 p234)(includes o218 p253)

(waiting o219)
(includes o219 p48)(includes o219 p90)(includes o219 p123)(includes o219 p178)(includes o219 p190)(includes o219 p227)(includes o219 p233)(includes o219 p235)(includes o219 p237)(includes o219 p261)

(waiting o220)
(includes o220 p30)(includes o220 p72)(includes o220 p83)(includes o220 p98)(includes o220 p113)(includes o220 p122)(includes o220 p126)(includes o220 p130)(includes o220 p131)(includes o220 p146)(includes o220 p202)

(waiting o221)
(includes o221 p77)(includes o221 p100)(includes o221 p176)

(waiting o222)
(includes o222 p130)(includes o222 p152)(includes o222 p255)

(waiting o223)
(includes o223 p33)(includes o223 p42)(includes o223 p72)(includes o223 p74)(includes o223 p97)(includes o223 p133)(includes o223 p166)

(waiting o224)
(includes o224 p3)(includes o224 p50)(includes o224 p96)(includes o224 p127)(includes o224 p178)(includes o224 p202)(includes o224 p232)(includes o224 p248)

(waiting o225)
(includes o225 p46)(includes o225 p84)(includes o225 p148)(includes o225 p243)(includes o225 p258)

(waiting o226)
(includes o226 p13)(includes o226 p82)(includes o226 p99)(includes o226 p106)(includes o226 p140)

(waiting o227)
(includes o227 p8)(includes o227 p27)(includes o227 p50)(includes o227 p110)(includes o227 p138)(includes o227 p187)(includes o227 p204)(includes o227 p253)

(waiting o228)
(includes o228 p15)(includes o228 p117)(includes o228 p118)(includes o228 p168)(includes o228 p175)(includes o228 p198)(includes o228 p199)(includes o228 p209)(includes o228 p218)(includes o228 p258)

(waiting o229)
(includes o229 p80)(includes o229 p84)(includes o229 p118)(includes o229 p219)(includes o229 p248)(includes o229 p264)

(waiting o230)
(includes o230 p12)(includes o230 p33)(includes o230 p138)(includes o230 p195)(includes o230 p234)

(waiting o231)
(includes o231 p5)(includes o231 p91)(includes o231 p113)(includes o231 p228)(includes o231 p258)

(waiting o232)
(includes o232 p31)(includes o232 p36)(includes o232 p45)(includes o232 p112)(includes o232 p116)(includes o232 p149)

(waiting o233)
(includes o233 p51)(includes o233 p80)(includes o233 p81)(includes o233 p115)(includes o233 p117)(includes o233 p125)(includes o233 p134)(includes o233 p135)(includes o233 p158)(includes o233 p216)(includes o233 p243)(includes o233 p265)

(waiting o234)
(includes o234 p1)(includes o234 p51)(includes o234 p82)(includes o234 p108)(includes o234 p111)(includes o234 p134)(includes o234 p177)(includes o234 p211)(includes o234 p239)(includes o234 p253)

(waiting o235)
(includes o235 p56)(includes o235 p59)(includes o235 p64)(includes o235 p80)(includes o235 p115)(includes o235 p236)

(waiting o236)
(includes o236 p80)(includes o236 p169)(includes o236 p213)

(waiting o237)
(includes o237 p1)(includes o237 p6)(includes o237 p19)(includes o237 p32)(includes o237 p63)(includes o237 p68)(includes o237 p95)(includes o237 p196)

(waiting o238)
(includes o238 p5)(includes o238 p21)(includes o238 p43)(includes o238 p95)(includes o238 p144)(includes o238 p177)(includes o238 p267)

(waiting o239)
(includes o239 p60)(includes o239 p96)(includes o239 p124)(includes o239 p174)(includes o239 p221)(includes o239 p231)(includes o239 p268)

(waiting o240)
(includes o240 p68)(includes o240 p116)(includes o240 p194)

(waiting o241)
(includes o241 p80)(includes o241 p82)(includes o241 p165)

(waiting o242)
(includes o242 p36)(includes o242 p54)(includes o242 p85)(includes o242 p109)(includes o242 p146)(includes o242 p214)(includes o242 p228)(includes o242 p233)(includes o242 p241)

(waiting o243)
(includes o243 p8)(includes o243 p21)(includes o243 p24)(includes o243 p83)(includes o243 p93)(includes o243 p113)(includes o243 p149)(includes o243 p156)(includes o243 p163)(includes o243 p230)(includes o243 p233)

(waiting o244)
(includes o244 p185)

(waiting o245)
(includes o245 p152)(includes o245 p169)(includes o245 p183)

(waiting o246)
(includes o246 p102)(includes o246 p107)(includes o246 p241)(includes o246 p255)

(waiting o247)
(includes o247 p158)(includes o247 p172)(includes o247 p197)(includes o247 p202)

(waiting o248)
(includes o248 p12)(includes o248 p20)(includes o248 p36)(includes o248 p54)(includes o248 p63)(includes o248 p73)(includes o248 p102)(includes o248 p134)(includes o248 p136)(includes o248 p178)(includes o248 p215)

(waiting o249)
(includes o249 p15)(includes o249 p104)(includes o249 p108)

(waiting o250)
(includes o250 p30)(includes o250 p79)(includes o250 p138)

(waiting o251)
(includes o251 p37)(includes o251 p119)(includes o251 p159)(includes o251 p172)(includes o251 p212)(includes o251 p242)

(waiting o252)
(includes o252 p7)(includes o252 p96)(includes o252 p135)(includes o252 p145)(includes o252 p173)(includes o252 p197)

(waiting o253)
(includes o253 p3)(includes o253 p45)(includes o253 p52)(includes o253 p71)(includes o253 p180)(includes o253 p200)

(waiting o254)
(includes o254 p27)(includes o254 p41)(includes o254 p128)(includes o254 p148)(includes o254 p205)(includes o254 p209)(includes o254 p233)

(waiting o255)
(includes o255 p43)(includes o255 p61)(includes o255 p78)(includes o255 p107)(includes o255 p133)(includes o255 p221)(includes o255 p229)(includes o255 p268)

(waiting o256)
(includes o256 p81)(includes o256 p126)(includes o256 p133)(includes o256 p181)(includes o256 p186)(includes o256 p207)(includes o256 p218)

(waiting o257)
(includes o257 p11)(includes o257 p60)(includes o257 p64)(includes o257 p79)(includes o257 p96)(includes o257 p186)(includes o257 p197)(includes o257 p249)

(waiting o258)
(includes o258 p9)(includes o258 p23)(includes o258 p77)(includes o258 p111)(includes o258 p117)(includes o258 p163)(includes o258 p190)(includes o258 p220)

(waiting o259)
(includes o259 p41)(includes o259 p71)(includes o259 p76)(includes o259 p159)(includes o259 p163)(includes o259 p164)(includes o259 p172)(includes o259 p265)

(waiting o260)
(includes o260 p10)(includes o260 p40)(includes o260 p61)(includes o260 p103)

(waiting o261)
(includes o261 p43)(includes o261 p105)(includes o261 p108)(includes o261 p166)(includes o261 p187)(includes o261 p239)

(waiting o262)
(includes o262 p246)

(waiting o263)
(includes o263 p32)(includes o263 p82)(includes o263 p135)

(waiting o264)
(includes o264 p92)(includes o264 p99)(includes o264 p228)(includes o264 p262)

(waiting o265)
(includes o265 p26)(includes o265 p61)(includes o265 p84)(includes o265 p215)(includes o265 p266)

(waiting o266)
(includes o266 p28)(includes o266 p59)(includes o266 p71)(includes o266 p97)(includes o266 p126)(includes o266 p127)(includes o266 p187)(includes o266 p198)(includes o266 p214)

(waiting o267)
(includes o267 p24)(includes o267 p145)

(waiting o268)
(includes o268 p16)(includes o268 p30)(includes o268 p82)(includes o268 p89)(includes o268 p170)

(waiting o269)
(includes o269 p56)(includes o269 p86)(includes o269 p146)(includes o269 p150)(includes o269 p151)(includes o269 p153)(includes o269 p207)(includes o269 p210)(includes o269 p232)

(waiting o270)
(includes o270 p11)(includes o270 p24)(includes o270 p56)(includes o270 p120)(includes o270 p229)

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

