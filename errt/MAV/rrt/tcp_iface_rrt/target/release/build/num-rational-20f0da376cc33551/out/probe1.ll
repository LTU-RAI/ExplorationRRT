; ModuleID = 'probe1.98c39414-cgu.0'
source_filename = "probe1.98c39414-cgu.0"
target datalayout = "e-m:e-p270:32:32-p271:32:32-p272:64:64-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

%"core::fmt::Arguments<'_>" = type { { ptr, i64 }, { ptr, i64 }, { ptr, i64 } }
%"alloc::string::String" = type { %"alloc::vec::Vec<u8>" }
%"alloc::vec::Vec<u8>" = type { { i64, ptr }, i64 }
%"core::ptr::metadata::PtrRepr<[u8]>" = type { [2 x i64] }
%"alloc::alloc::Global" = type {}
%"core::option::Option<(core::ptr::non_null::NonNull<u8>, core::alloc::layout::Layout)>" = type { [2 x i64], i64 }

@alloc_7d3945583a90247e161598063e3b9526 = private unnamed_addr constant <{}> zeroinitializer, align 8
@alloc_09d11aa498739cbf0519d318f9792c9b = private unnamed_addr constant <{ [12 x i8] }> <{ [12 x i8] c"invalid args" }>, align 1
@alloc_71b99a1804d93c013f301ec59bc480be = private unnamed_addr constant <{ ptr, [8 x i8] }> <{ ptr @alloc_09d11aa498739cbf0519d318f9792c9b, [8 x i8] c"\0C\00\00\00\00\00\00\00" }>, align 8
@alloc_e2222df79fea71f2539d68a3a6ae8b04 = private unnamed_addr constant <{ [75 x i8] }> <{ [75 x i8] c"/rustc/84c898d65adf2f39a5a98507f1fe0ce10a2b8dbc/library/core/src/fmt/mod.rs" }>, align 1
@alloc_ffaf8578af742fcd9190283b2acd56d3 = private unnamed_addr constant <{ ptr, [16 x i8] }> <{ ptr @alloc_e2222df79fea71f2539d68a3a6ae8b04, [16 x i8] c"K\00\00\00\00\00\00\00\93\01\00\00\0D\00\00\00" }>, align 8
@alloc_1ddcd833969caa2a318f2d1b0a4897ab = private unnamed_addr constant <{ [80 x i8] }> <{ [80 x i8] c"/rustc/84c898d65adf2f39a5a98507f1fe0ce10a2b8dbc/library/core/src/alloc/layout.rs" }>, align 1
@alloc_6dd5f20633b30d978a88c2668d4a1b10 = private unnamed_addr constant <{ ptr, [16 x i8] }> <{ ptr @alloc_1ddcd833969caa2a318f2d1b0a4897ab, [16 x i8] c"P\00\00\00\00\00\00\00\C4\01\00\00)\00\00\00" }>, align 8
@str.0 = internal constant [25 x i8] c"attempt to divide by zero"
@alloc_997d7ac396f89f2a981093fc6d33b686 = private unnamed_addr constant <{ ptr, [8 x i8] }> <{ ptr @alloc_7d3945583a90247e161598063e3b9526, [8 x i8] zeroinitializer }>, align 8
@alloc_3d6223e1ee89533f014a7f6f8d8992fe = private unnamed_addr constant <{ [8 x i8] }> zeroinitializer, align 8

; <core::ptr::non_null::NonNull<T> as core::convert::From<core::ptr::unique::Unique<T>>>::from
; Function Attrs: inlinehint nonlazybind uwtable
define ptr @"_ZN119_$LT$core..ptr..non_null..NonNull$LT$T$GT$$u20$as$u20$core..convert..From$LT$core..ptr..unique..Unique$LT$T$GT$$GT$$GT$4from17hbb06d653e9d30030E"(ptr %unique) unnamed_addr #0 {
start:
  %0 = alloca ptr, align 8
  store ptr %unique, ptr %0, align 8
  %1 = load ptr, ptr %0, align 8, !nonnull !2, !noundef !2
  ret ptr %1
}

; core::fmt::ArgumentV1::new_lower_exp
; Function Attrs: inlinehint nonlazybind uwtable
define { ptr, ptr } @_ZN4core3fmt10ArgumentV113new_lower_exp17h8ac11525faa56e76E(ptr align 8 %x) unnamed_addr #0 {
start:
  %0 = alloca ptr, align 8
  %1 = alloca ptr, align 8
  %2 = alloca { ptr, ptr }, align 8
  store ptr @"_ZN4core3fmt3num3imp55_$LT$impl$u20$core..fmt..LowerExp$u20$for$u20$isize$GT$3fmt17h02e9c410ed26b14bE", ptr %1, align 8
  %_3 = load ptr, ptr %1, align 8, !nonnull !2, !noundef !2
  store ptr %x, ptr %0, align 8
  %_4 = load ptr, ptr %0, align 8, !nonnull !2, !align !3, !noundef !2
  store ptr %_4, ptr %2, align 8
  %3 = getelementptr inbounds { ptr, ptr }, ptr %2, i32 0, i32 1
  store ptr %_3, ptr %3, align 8
  %4 = getelementptr inbounds { ptr, ptr }, ptr %2, i32 0, i32 0
  %5 = load ptr, ptr %4, align 8, !nonnull !2, !align !3, !noundef !2
  %6 = getelementptr inbounds { ptr, ptr }, ptr %2, i32 0, i32 1
  %7 = load ptr, ptr %6, align 8, !nonnull !2, !noundef !2
  %8 = insertvalue { ptr, ptr } undef, ptr %5, 0
  %9 = insertvalue { ptr, ptr } %8, ptr %7, 1
  ret { ptr, ptr } %9
}

; core::fmt::Arguments::as_str
; Function Attrs: inlinehint nonlazybind uwtable
define internal { ptr, i64 } @_ZN4core3fmt9Arguments6as_str17h25aa4fd788f15230E(ptr align 8 %self) unnamed_addr #0 {
start:
  %_2 = alloca { { ptr, i64 }, { ptr, i64 } }, align 8
  %0 = alloca { ptr, i64 }, align 8
  %1 = getelementptr inbounds %"core::fmt::Arguments<'_>", ptr %self, i32 0, i32 1
  %2 = getelementptr inbounds { ptr, i64 }, ptr %1, i32 0, i32 0
  %_3.0 = load ptr, ptr %2, align 8, !nonnull !2, !align !4, !noundef !2
  %3 = getelementptr inbounds { ptr, i64 }, ptr %1, i32 0, i32 1
  %_3.1 = load i64, ptr %3, align 8, !noundef !2
  %4 = getelementptr inbounds %"core::fmt::Arguments<'_>", ptr %self, i32 0, i32 2
  %5 = getelementptr inbounds { ptr, i64 }, ptr %4, i32 0, i32 0
  %_4.0 = load ptr, ptr %5, align 8, !nonnull !2, !align !4, !noundef !2
  %6 = getelementptr inbounds { ptr, i64 }, ptr %4, i32 0, i32 1
  %_4.1 = load i64, ptr %6, align 8, !noundef !2
  %7 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 0
  store ptr %_3.0, ptr %7, align 8
  %8 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 1
  store i64 %_3.1, ptr %8, align 8
  %9 = getelementptr inbounds { { ptr, i64 }, { ptr, i64 } }, ptr %_2, i32 0, i32 1
  %10 = getelementptr inbounds { ptr, i64 }, ptr %9, i32 0, i32 0
  store ptr %_4.0, ptr %10, align 8
  %11 = getelementptr inbounds { ptr, i64 }, ptr %9, i32 0, i32 1
  store i64 %_4.1, ptr %11, align 8
  %12 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 0
  %_19.0 = load ptr, ptr %12, align 8, !nonnull !2, !align !4, !noundef !2
  %13 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 1
  %_19.1 = load i64, ptr %13, align 8, !noundef !2
  %_16 = icmp eq i64 %_19.1, 0
  br i1 %_16, label %bb1, label %bb3

bb3:                                              ; preds = %start
  %14 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 0
  %_21.0 = load ptr, ptr %14, align 8, !nonnull !2, !align !4, !noundef !2
  %15 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 1
  %_21.1 = load i64, ptr %15, align 8, !noundef !2
  %_13 = icmp eq i64 %_21.1, 1
  br i1 %_13, label %bb4, label %bb2

bb1:                                              ; preds = %start
  %16 = getelementptr inbounds { { ptr, i64 }, { ptr, i64 } }, ptr %_2, i32 0, i32 1
  %17 = getelementptr inbounds { ptr, i64 }, ptr %16, i32 0, i32 0
  %_20.0 = load ptr, ptr %17, align 8, !nonnull !2, !align !4, !noundef !2
  %18 = getelementptr inbounds { ptr, i64 }, ptr %16, i32 0, i32 1
  %_20.1 = load i64, ptr %18, align 8, !noundef !2
  %_7 = icmp eq i64 %_20.1, 0
  br i1 %_7, label %bb5, label %bb2

bb2:                                              ; preds = %bb4, %bb3, %bb1
  store ptr null, ptr %0, align 8
  br label %bb7

bb5:                                              ; preds = %bb1
  %19 = getelementptr inbounds { ptr, i64 }, ptr %0, i32 0, i32 0
  store ptr @alloc_7d3945583a90247e161598063e3b9526, ptr %19, align 8
  %20 = getelementptr inbounds { ptr, i64 }, ptr %0, i32 0, i32 1
  store i64 0, ptr %20, align 8
  br label %bb7

bb7:                                              ; preds = %bb2, %bb6, %bb5
  %21 = getelementptr inbounds { ptr, i64 }, ptr %0, i32 0, i32 0
  %22 = load ptr, ptr %21, align 8, !align !3, !noundef !2
  %23 = getelementptr inbounds { ptr, i64 }, ptr %0, i32 0, i32 1
  %24 = load i64, ptr %23, align 8
  %25 = insertvalue { ptr, i64 } undef, ptr %22, 0
  %26 = insertvalue { ptr, i64 } %25, i64 %24, 1
  ret { ptr, i64 } %26

bb4:                                              ; preds = %bb3
  %27 = getelementptr inbounds { { ptr, i64 }, { ptr, i64 } }, ptr %_2, i32 0, i32 1
  %28 = getelementptr inbounds { ptr, i64 }, ptr %27, i32 0, i32 0
  %_22.0 = load ptr, ptr %28, align 8, !nonnull !2, !align !4, !noundef !2
  %29 = getelementptr inbounds { ptr, i64 }, ptr %27, i32 0, i32 1
  %_22.1 = load i64, ptr %29, align 8, !noundef !2
  %_10 = icmp eq i64 %_22.1, 0
  br i1 %_10, label %bb6, label %bb2

bb6:                                              ; preds = %bb4
  %30 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 0
  %_23.0 = load ptr, ptr %30, align 8, !nonnull !2, !align !4, !noundef !2
  %31 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 1
  %_23.1 = load i64, ptr %31, align 8, !noundef !2
  %s = getelementptr inbounds [0 x { ptr, i64 }], ptr %_23.0, i64 0, i64 0
  %32 = getelementptr inbounds { ptr, i64 }, ptr %s, i32 0, i32 0
  %_24.0 = load ptr, ptr %32, align 8, !nonnull !2, !align !3, !noundef !2
  %33 = getelementptr inbounds { ptr, i64 }, ptr %s, i32 0, i32 1
  %_24.1 = load i64, ptr %33, align 8, !noundef !2
  %34 = getelementptr inbounds { ptr, i64 }, ptr %0, i32 0, i32 0
  store ptr %_24.0, ptr %34, align 8
  %35 = getelementptr inbounds { ptr, i64 }, ptr %0, i32 0, i32 1
  store i64 %_24.1, ptr %35, align 8
  br label %bb7
}

; core::fmt::Arguments::new_v1
; Function Attrs: inlinehint nonlazybind uwtable
define internal void @_ZN4core3fmt9Arguments6new_v117hb0661b8fc4ebe3c8E(ptr sret(%"core::fmt::Arguments<'_>") %0, ptr align 8 %pieces.0, i64 %pieces.1, ptr align 8 %args.0, i64 %args.1) unnamed_addr #0 {
start:
  %_15 = alloca { ptr, i64 }, align 8
  %_12 = alloca %"core::fmt::Arguments<'_>", align 8
  %_3 = alloca i8, align 1
  %_4 = icmp ult i64 %pieces.1, %args.1
  br i1 %_4, label %bb1, label %bb2

bb2:                                              ; preds = %start
  %_9 = add i64 %args.1, 1
  %_7 = icmp ugt i64 %pieces.1, %_9
  %1 = zext i1 %_7 to i8
  store i8 %1, ptr %_3, align 1
  br label %bb3

bb1:                                              ; preds = %start
  store i8 1, ptr %_3, align 1
  br label %bb3

bb3:                                              ; preds = %bb2, %bb1
  %2 = load i8, ptr %_3, align 1, !range !5, !noundef !2
  %3 = trunc i8 %2 to i1
  br i1 %3, label %bb4, label %bb6

bb6:                                              ; preds = %bb3
  store ptr null, ptr %_15, align 8
  %4 = getelementptr inbounds %"core::fmt::Arguments<'_>", ptr %0, i32 0, i32 1
  %5 = getelementptr inbounds { ptr, i64 }, ptr %4, i32 0, i32 0
  store ptr %pieces.0, ptr %5, align 8
  %6 = getelementptr inbounds { ptr, i64 }, ptr %4, i32 0, i32 1
  store i64 %pieces.1, ptr %6, align 8
  %7 = getelementptr inbounds { ptr, i64 }, ptr %_15, i32 0, i32 0
  %8 = load ptr, ptr %7, align 8, !align !4, !noundef !2
  %9 = getelementptr inbounds { ptr, i64 }, ptr %_15, i32 0, i32 1
  %10 = load i64, ptr %9, align 8
  %11 = getelementptr inbounds { ptr, i64 }, ptr %0, i32 0, i32 0
  store ptr %8, ptr %11, align 8
  %12 = getelementptr inbounds { ptr, i64 }, ptr %0, i32 0, i32 1
  store i64 %10, ptr %12, align 8
  %13 = getelementptr inbounds %"core::fmt::Arguments<'_>", ptr %0, i32 0, i32 2
  %14 = getelementptr inbounds { ptr, i64 }, ptr %13, i32 0, i32 0
  store ptr %args.0, ptr %14, align 8
  %15 = getelementptr inbounds { ptr, i64 }, ptr %13, i32 0, i32 1
  store i64 %args.1, ptr %15, align 8
  ret void

bb4:                                              ; preds = %bb3
; call core::fmt::Arguments::new_v1
  call void @_ZN4core3fmt9Arguments6new_v117hb0661b8fc4ebe3c8E(ptr sret(%"core::fmt::Arguments<'_>") %_12, ptr align 8 @alloc_71b99a1804d93c013f301ec59bc480be, i64 1, ptr align 8 @alloc_7d3945583a90247e161598063e3b9526, i64 0)
; call core::panicking::panic_fmt
  call void @_ZN4core9panicking9panic_fmt17hf33a1475b4dc5c3eE(ptr %_12, ptr align 8 @alloc_ffaf8578af742fcd9190283b2acd56d3) #12
  unreachable
}

; core::ops::function::FnOnce::call_once
; Function Attrs: inlinehint nonlazybind uwtable
define internal void @_ZN4core3ops8function6FnOnce9call_once17h28e68220631ef774E(ptr sret(%"alloc::string::String") %0, ptr align 1 %1, i64 %2) unnamed_addr #0 {
start:
  %_2 = alloca { ptr, i64 }, align 8
  %3 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 0
  store ptr %1, ptr %3, align 8
  %4 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 1
  store i64 %2, ptr %4, align 8
  %5 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 0
  %6 = load ptr, ptr %5, align 8, !nonnull !2, !align !3, !noundef !2
  %7 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 1
  %8 = load i64, ptr %7, align 8, !noundef !2
; call alloc::str::<impl alloc::borrow::ToOwned for str>::to_owned
  call void @"_ZN5alloc3str56_$LT$impl$u20$alloc..borrow..ToOwned$u20$for$u20$str$GT$8to_owned17h4b90106d4b08620fE"(ptr sret(%"alloc::string::String") %0, ptr align 1 %6, i64 %8)
  ret void
}

; core::ptr::drop_in_place<alloc::string::String>
; Function Attrs: nonlazybind uwtable
define void @"_ZN4core3ptr42drop_in_place$LT$alloc..string..String$GT$17hb4af148fd5621c75E"(ptr %_1) unnamed_addr #1 {
start:
; call core::ptr::drop_in_place<alloc::vec::Vec<u8>>
  call void @"_ZN4core3ptr46drop_in_place$LT$alloc..vec..Vec$LT$u8$GT$$GT$17h22cdae9a09d54f76E"(ptr %_1)
  ret void
}

; core::ptr::drop_in_place<alloc::vec::Vec<u8>>
; Function Attrs: nonlazybind uwtable
define void @"_ZN4core3ptr46drop_in_place$LT$alloc..vec..Vec$LT$u8$GT$$GT$17h22cdae9a09d54f76E"(ptr %_1) unnamed_addr #1 personality ptr @rust_eh_personality {
start:
  %0 = alloca { ptr, i32 }, align 8
; invoke <alloc::vec::Vec<T,A> as core::ops::drop::Drop>::drop
  invoke void @"_ZN70_$LT$alloc..vec..Vec$LT$T$C$A$GT$$u20$as$u20$core..ops..drop..Drop$GT$4drop17h4ec93900252c0b8dE"(ptr align 8 %_1)
          to label %bb4 unwind label %cleanup

bb3:                                              ; preds = %cleanup
; invoke core::ptr::drop_in_place<alloc::raw_vec::RawVec<u8>>
  invoke void @"_ZN4core3ptr53drop_in_place$LT$alloc..raw_vec..RawVec$LT$u8$GT$$GT$17hccd3ddcc2cb4949eE"(ptr %_1) #13
          to label %bb1 unwind label %abort

cleanup:                                          ; preds = %start
  %1 = landingpad { ptr, i32 }
          cleanup
  %2 = extractvalue { ptr, i32 } %1, 0
  %3 = extractvalue { ptr, i32 } %1, 1
  %4 = getelementptr inbounds { ptr, i32 }, ptr %0, i32 0, i32 0
  store ptr %2, ptr %4, align 8
  %5 = getelementptr inbounds { ptr, i32 }, ptr %0, i32 0, i32 1
  store i32 %3, ptr %5, align 8
  br label %bb3

bb4:                                              ; preds = %start
; call core::ptr::drop_in_place<alloc::raw_vec::RawVec<u8>>
  call void @"_ZN4core3ptr53drop_in_place$LT$alloc..raw_vec..RawVec$LT$u8$GT$$GT$17hccd3ddcc2cb4949eE"(ptr %_1)
  ret void

abort:                                            ; preds = %bb3
  %6 = landingpad { ptr, i32 }
          cleanup
  %7 = extractvalue { ptr, i32 } %6, 0
  %8 = extractvalue { ptr, i32 } %6, 1
; call core::panicking::panic_cannot_unwind
  call void @_ZN4core9panicking19panic_cannot_unwind17hd123d9c71473dcdaE() #14
  unreachable

bb1:                                              ; preds = %bb3
  %9 = load ptr, ptr %0, align 8, !noundef !2
  %10 = getelementptr inbounds { ptr, i32 }, ptr %0, i32 0, i32 1
  %11 = load i32, ptr %10, align 8, !noundef !2
  %12 = insertvalue { ptr, i32 } undef, ptr %9, 0
  %13 = insertvalue { ptr, i32 } %12, i32 %11, 1
  resume { ptr, i32 } %13
}

; core::ptr::drop_in_place<alloc::raw_vec::RawVec<u8>>
; Function Attrs: nonlazybind uwtable
define void @"_ZN4core3ptr53drop_in_place$LT$alloc..raw_vec..RawVec$LT$u8$GT$$GT$17hccd3ddcc2cb4949eE"(ptr %_1) unnamed_addr #1 {
start:
; call <alloc::raw_vec::RawVec<T,A> as core::ops::drop::Drop>::drop
  call void @"_ZN77_$LT$alloc..raw_vec..RawVec$LT$T$C$A$GT$$u20$as$u20$core..ops..drop..Drop$GT$4drop17h6334db8ef1f3288eE"(ptr align 8 %_1)
  ret void
}

; core::alloc::layout::Layout::array::inner
; Function Attrs: inlinehint nonlazybind uwtable
define internal { i64, i64 } @_ZN4core5alloc6layout6Layout5array5inner17h286ae9a02469cf04E(i64 %element_size, i64 %align, i64 %n) unnamed_addr #0 {
start:
  %0 = alloca i64, align 8
  %_20 = alloca i64, align 8
  %_16 = alloca i64, align 8
  %_11 = alloca { i64, i64 }, align 8
  %_4 = alloca i8, align 1
  %1 = alloca { i64, i64 }, align 8
  %2 = icmp eq i64 %element_size, 0
  br i1 %2, label %bb1, label %bb2

bb1:                                              ; preds = %start
  store i8 0, ptr %_4, align 1
  br label %bb3

bb2:                                              ; preds = %start
  store i64 %align, ptr %_16, align 8
  %_17 = load i64, ptr %_16, align 8, !range !6, !noundef !2
  %_18 = icmp uge i64 -9223372036854775808, %_17
  call void @llvm.assume(i1 %_18)
  %_19 = icmp ule i64 1, %_17
  call void @llvm.assume(i1 %_19)
  %_14 = sub i64 %_17, 1
  %_7 = sub i64 9223372036854775807, %_14
  %_8 = icmp eq i64 %element_size, 0
  %3 = call i1 @llvm.expect.i1(i1 %_8, i1 false)
  br i1 %3, label %panic, label %bb4

bb4:                                              ; preds = %bb2
  %_6 = udiv i64 %_7, %element_size
  %_5 = icmp ugt i64 %n, %_6
  %4 = zext i1 %_5 to i8
  store i8 %4, ptr %_4, align 1
  br label %bb3

panic:                                            ; preds = %bb2
; call core::panicking::panic
  call void @_ZN4core9panicking5panic17h9533b2fee90b999eE(ptr align 1 @str.0, i64 25, ptr align 8 @alloc_6dd5f20633b30d978a88c2668d4a1b10) #12
  unreachable

bb3:                                              ; preds = %bb1, %bb4
  %5 = load i8, ptr %_4, align 1, !range !5, !noundef !2
  %6 = trunc i8 %5 to i1
  br i1 %6, label %bb5, label %bb6

bb6:                                              ; preds = %bb3
  %array_size = mul i64 %element_size, %n
  store i64 %align, ptr %_20, align 8
  %_21 = load i64, ptr %_20, align 8, !range !6, !noundef !2
  %_22 = icmp uge i64 -9223372036854775808, %_21
  call void @llvm.assume(i1 %_22)
  %_23 = icmp ule i64 1, %_21
  call void @llvm.assume(i1 %_23)
  store i64 %_21, ptr %0, align 8
  %_24 = load i64, ptr %0, align 8, !range !6, !noundef !2
  store i64 %array_size, ptr %_11, align 8
  %7 = getelementptr inbounds { i64, i64 }, ptr %_11, i32 0, i32 1
  store i64 %_24, ptr %7, align 8
  %8 = getelementptr inbounds { i64, i64 }, ptr %_11, i32 0, i32 0
  %9 = load i64, ptr %8, align 8, !noundef !2
  %10 = getelementptr inbounds { i64, i64 }, ptr %_11, i32 0, i32 1
  %11 = load i64, ptr %10, align 8, !range !6, !noundef !2
  %12 = getelementptr inbounds { i64, i64 }, ptr %1, i32 0, i32 0
  store i64 %9, ptr %12, align 8
  %13 = getelementptr inbounds { i64, i64 }, ptr %1, i32 0, i32 1
  store i64 %11, ptr %13, align 8
  br label %bb7

bb5:                                              ; preds = %bb3
  %14 = getelementptr inbounds { i64, i64 }, ptr %1, i32 0, i32 1
  store i64 0, ptr %14, align 8
  br label %bb7

bb7:                                              ; preds = %bb6, %bb5
  %15 = getelementptr inbounds { i64, i64 }, ptr %1, i32 0, i32 0
  %16 = load i64, ptr %15, align 8
  %17 = getelementptr inbounds { i64, i64 }, ptr %1, i32 0, i32 1
  %18 = load i64, ptr %17, align 8, !range !7, !noundef !2
  %19 = insertvalue { i64, i64 } undef, i64 %16, 0
  %20 = insertvalue { i64, i64 } %19, i64 %18, 1
  ret { i64, i64 } %20
}

; core::option::Option<T>::map_or_else
; Function Attrs: inlinehint nonlazybind uwtable
define void @"_ZN4core6option15Option$LT$T$GT$11map_or_else17h76f63214bb1974d4E"(ptr sret(%"alloc::string::String") %0, ptr align 1 %1, i64 %2, ptr align 8 %default) unnamed_addr #0 personality ptr @rust_eh_personality {
start:
  %3 = alloca { ptr, i32 }, align 8
  %_11 = alloca i8, align 1
  %_10 = alloca i8, align 1
  %_7 = alloca { ptr, i64 }, align 8
  %self = alloca { ptr, i64 }, align 8
  %4 = getelementptr inbounds { ptr, i64 }, ptr %self, i32 0, i32 0
  store ptr %1, ptr %4, align 8
  %5 = getelementptr inbounds { ptr, i64 }, ptr %self, i32 0, i32 1
  store i64 %2, ptr %5, align 8
  store i8 1, ptr %_11, align 1
  store i8 1, ptr %_10, align 1
  %6 = load ptr, ptr %self, align 8, !noundef !2
  %7 = ptrtoint ptr %6 to i64
  %8 = icmp eq i64 %7, 0
  %_4 = select i1 %8, i64 0, i64 1
  %9 = icmp eq i64 %_4, 0
  br i1 %9, label %bb1, label %bb3

bb1:                                              ; preds = %start
  store i8 0, ptr %_11, align 1
; invoke alloc::fmt::format::{{closure}}
  invoke void @"_ZN5alloc3fmt6format28_$u7b$$u7b$closure$u7d$$u7d$17haf4309b5dac31009E"(ptr sret(%"alloc::string::String") %0, ptr align 8 %default)
          to label %bb5 unwind label %cleanup

bb3:                                              ; preds = %start
  %10 = getelementptr inbounds { ptr, i64 }, ptr %self, i32 0, i32 0
  %t.0 = load ptr, ptr %10, align 8, !nonnull !2, !align !3, !noundef !2
  %11 = getelementptr inbounds { ptr, i64 }, ptr %self, i32 0, i32 1
  %t.1 = load i64, ptr %11, align 8, !noundef !2
  store i8 0, ptr %_10, align 1
  %12 = getelementptr inbounds { ptr, i64 }, ptr %_7, i32 0, i32 0
  store ptr %t.0, ptr %12, align 8
  %13 = getelementptr inbounds { ptr, i64 }, ptr %_7, i32 0, i32 1
  store i64 %t.1, ptr %13, align 8
  %14 = getelementptr inbounds { ptr, i64 }, ptr %_7, i32 0, i32 0
  %15 = load ptr, ptr %14, align 8, !nonnull !2, !align !3, !noundef !2
  %16 = getelementptr inbounds { ptr, i64 }, ptr %_7, i32 0, i32 1
  %17 = load i64, ptr %16, align 8, !noundef !2
; invoke core::ops::function::FnOnce::call_once
  invoke void @_ZN4core3ops8function6FnOnce9call_once17h28e68220631ef774E(ptr sret(%"alloc::string::String") %0, ptr align 1 %15, i64 %17)
          to label %bb4 unwind label %cleanup

bb2:                                              ; No predecessors!
  unreachable

bb14:                                             ; preds = %cleanup
  %18 = load i8, ptr %_10, align 1, !range !5, !noundef !2
  %19 = trunc i8 %18 to i1
  br i1 %19, label %bb13, label %bb8

cleanup:                                          ; preds = %bb1, %bb3
  %20 = landingpad { ptr, i32 }
          cleanup
  %21 = extractvalue { ptr, i32 } %20, 0
  %22 = extractvalue { ptr, i32 } %20, 1
  %23 = getelementptr inbounds { ptr, i32 }, ptr %3, i32 0, i32 0
  store ptr %21, ptr %23, align 8
  %24 = getelementptr inbounds { ptr, i32 }, ptr %3, i32 0, i32 1
  store i32 %22, ptr %24, align 8
  br label %bb14

bb4:                                              ; preds = %bb3
  br label %bb11

bb11:                                             ; preds = %bb5, %bb4
  %25 = load i8, ptr %_10, align 1, !range !5, !noundef !2
  %26 = trunc i8 %25 to i1
  br i1 %26, label %bb10, label %bb6

bb5:                                              ; preds = %bb1
  br label %bb11

bb8:                                              ; preds = %bb13, %bb14
  %27 = load i8, ptr %_11, align 1, !range !5, !noundef !2
  %28 = trunc i8 %27 to i1
  br i1 %28, label %bb15, label %bb9

bb13:                                             ; preds = %bb14
  br label %bb8

bb6:                                              ; preds = %bb10, %bb11
  %29 = load i8, ptr %_11, align 1, !range !5, !noundef !2
  %30 = trunc i8 %29 to i1
  br i1 %30, label %bb12, label %bb7

bb10:                                             ; preds = %bb11
  br label %bb6

bb9:                                              ; preds = %bb15, %bb8
  %31 = load ptr, ptr %3, align 8, !noundef !2
  %32 = getelementptr inbounds { ptr, i32 }, ptr %3, i32 0, i32 1
  %33 = load i32, ptr %32, align 8, !noundef !2
  %34 = insertvalue { ptr, i32 } undef, ptr %31, 0
  %35 = insertvalue { ptr, i32 } %34, i32 %33, 1
  resume { ptr, i32 } %35

bb15:                                             ; preds = %bb8
  br label %bb9

bb7:                                              ; preds = %bb12, %bb6
  ret void

bb12:                                             ; preds = %bb6
  br label %bb7
}

; <T as core::convert::Into<U>>::into
; Function Attrs: nonlazybind uwtable
define ptr @"_ZN50_$LT$T$u20$as$u20$core..convert..Into$LT$U$GT$$GT$4into17h6d20ebae4fbb1ff4E"(ptr %self) unnamed_addr #1 {
start:
; call <core::ptr::non_null::NonNull<T> as core::convert::From<core::ptr::unique::Unique<T>>>::from
  %0 = call ptr @"_ZN119_$LT$core..ptr..non_null..NonNull$LT$T$GT$$u20$as$u20$core..convert..From$LT$core..ptr..unique..Unique$LT$T$GT$$GT$$GT$4from17hbb06d653e9d30030E"(ptr %self)
  ret ptr %0
}

; <T as alloc::slice::hack::ConvertVec>::to_vec
; Function Attrs: inlinehint nonlazybind uwtable
define void @"_ZN52_$LT$T$u20$as$u20$alloc..slice..hack..ConvertVec$GT$6to_vec17h9128de7e47161cabE"(ptr sret(%"alloc::vec::Vec<u8>") %v, ptr align 1 %s.0, i64 %s.1) unnamed_addr #0 personality ptr @rust_eh_personality {
start:
  %0 = alloca i64, align 8
  %1 = alloca { ptr, i32 }, align 8
  %_34 = alloca %"core::ptr::metadata::PtrRepr<[u8]>", align 8
  %_29 = alloca %"core::ptr::metadata::PtrRepr<[u8]>", align 8
  %_23 = alloca ptr, align 8
  %_13 = alloca %"core::ptr::metadata::PtrRepr<[u8]>", align 8
  %2 = getelementptr inbounds { ptr, i64 }, ptr %_13, i32 0, i32 0
  store ptr %s.0, ptr %2, align 8
  %3 = getelementptr inbounds { ptr, i64 }, ptr %_13, i32 0, i32 1
  store i64 %s.1, ptr %3, align 8
  %4 = getelementptr inbounds { ptr, i64 }, ptr %_13, i32 0, i32 1
  %capacity = load i64, ptr %4, align 8, !noundef !2
; invoke alloc::raw_vec::RawVec<T,A>::allocate_in
  %5 = invoke { i64, ptr } @"_ZN5alloc7raw_vec19RawVec$LT$T$C$A$GT$11allocate_in17h0e1e1f61f43e6e42E"(i64 %capacity, i1 zeroext false)
          to label %bb5 unwind label %cleanup

bb4:                                              ; preds = %bb1, %cleanup
  br i1 false, label %bb3, label %bb2

cleanup:                                          ; preds = %start
  %6 = landingpad { ptr, i32 }
          cleanup
  %7 = extractvalue { ptr, i32 } %6, 0
  %8 = extractvalue { ptr, i32 } %6, 1
  %9 = getelementptr inbounds { ptr, i32 }, ptr %1, i32 0, i32 0
  store ptr %7, ptr %9, align 8
  %10 = getelementptr inbounds { ptr, i32 }, ptr %1, i32 0, i32 1
  store i32 %8, ptr %10, align 8
  br label %bb4

bb5:                                              ; preds = %start
  %_14.0 = extractvalue { i64, ptr } %5, 0
  %_14.1 = extractvalue { i64, ptr } %5, 1
  %11 = getelementptr inbounds { i64, ptr }, ptr %v, i32 0, i32 0
  store i64 %_14.0, ptr %11, align 8
  %12 = getelementptr inbounds { i64, ptr }, ptr %v, i32 0, i32 1
  store ptr %_14.1, ptr %12, align 8
  %13 = getelementptr inbounds %"alloc::vec::Vec<u8>", ptr %v, i32 0, i32 1
  store i64 0, ptr %13, align 8
  %14 = getelementptr inbounds { i64, ptr }, ptr %v, i32 0, i32 1
  %self = load ptr, ptr %14, align 8, !nonnull !2, !noundef !2
  store ptr %self, ptr %_23, align 8
  %ptr = load ptr, ptr %_23, align 8, !noundef !2
  store ptr %ptr, ptr %0, align 8
  %_26 = load i64, ptr %0, align 8, !noundef !2
  br label %bb6

bb6:                                              ; preds = %bb5
  %_19 = icmp eq i64 %_26, 0
  %_18 = xor i1 %_19, true
  call void @llvm.assume(i1 %_18)
  %15 = getelementptr inbounds { ptr, i64 }, ptr %_29, i32 0, i32 0
  store ptr %s.0, ptr %15, align 8
  %16 = getelementptr inbounds { ptr, i64 }, ptr %_29, i32 0, i32 1
  store i64 %s.1, ptr %16, align 8
  %17 = getelementptr inbounds { ptr, i64 }, ptr %_29, i32 0, i32 1
  %count = load i64, ptr %17, align 8, !noundef !2
  %18 = mul i64 %count, 1
  call void @llvm.memcpy.p0.p0.i64(ptr align 1 %self, ptr align 1 %s.0, i64 %18, i1 false)
  %19 = getelementptr inbounds { ptr, i64 }, ptr %_34, i32 0, i32 0
  store ptr %s.0, ptr %19, align 8
  %20 = getelementptr inbounds { ptr, i64 }, ptr %_34, i32 0, i32 1
  store i64 %s.1, ptr %20, align 8
  %21 = getelementptr inbounds { ptr, i64 }, ptr %_34, i32 0, i32 1
  %new_len = load i64, ptr %21, align 8, !noundef !2
  %22 = getelementptr inbounds %"alloc::vec::Vec<u8>", ptr %v, i32 0, i32 1
  store i64 %new_len, ptr %22, align 8
  ret void

bb1:                                              ; No predecessors!
; invoke core::ptr::drop_in_place<alloc::vec::Vec<u8>>
  invoke void @"_ZN4core3ptr46drop_in_place$LT$alloc..vec..Vec$LT$u8$GT$$GT$17h22cdae9a09d54f76E"(ptr %v) #13
          to label %bb4 unwind label %abort

abort:                                            ; preds = %bb1
  %23 = landingpad { ptr, i32 }
          cleanup
  %24 = extractvalue { ptr, i32 } %23, 0
  %25 = extractvalue { ptr, i32 } %23, 1
; call core::panicking::panic_cannot_unwind
  call void @_ZN4core9panicking19panic_cannot_unwind17hd123d9c71473dcdaE() #14
  unreachable

bb2:                                              ; preds = %bb3, %bb4
  %26 = load ptr, ptr %1, align 8, !noundef !2
  %27 = getelementptr inbounds { ptr, i32 }, ptr %1, i32 0, i32 1
  %28 = load i32, ptr %27, align 8, !noundef !2
  %29 = insertvalue { ptr, i32 } undef, ptr %26, 0
  %30 = insertvalue { ptr, i32 } %29, i32 %28, 1
  resume { ptr, i32 } %30

bb3:                                              ; preds = %bb4
  br label %bb2
}

; alloc::fmt::format
; Function Attrs: inlinehint nonlazybind uwtable
define internal void @_ZN5alloc3fmt6format17hf71f9424c95d3754E(ptr sret(%"alloc::string::String") %0, ptr %args) unnamed_addr #0 {
start:
  %_4 = alloca ptr, align 8
; call core::fmt::Arguments::as_str
  %1 = call { ptr, i64 } @_ZN4core3fmt9Arguments6as_str17h25aa4fd788f15230E(ptr align 8 %args)
  %_2.0 = extractvalue { ptr, i64 } %1, 0
  %_2.1 = extractvalue { ptr, i64 } %1, 1
  store ptr %args, ptr %_4, align 8
  %2 = load ptr, ptr %_4, align 8, !nonnull !2, !align !4, !noundef !2
; call core::option::Option<T>::map_or_else
  call void @"_ZN4core6option15Option$LT$T$GT$11map_or_else17h76f63214bb1974d4E"(ptr sret(%"alloc::string::String") %0, ptr align 1 %_2.0, i64 %_2.1, ptr align 8 %2)
  ret void
}

; alloc::fmt::format::{{closure}}
; Function Attrs: inlinehint nonlazybind uwtable
define void @"_ZN5alloc3fmt6format28_$u7b$$u7b$closure$u7d$$u7d$17haf4309b5dac31009E"(ptr sret(%"alloc::string::String") %0, ptr align 8 %1) unnamed_addr #0 {
start:
  %_2 = alloca %"core::fmt::Arguments<'_>", align 8
  %_1 = alloca ptr, align 8
  store ptr %1, ptr %_1, align 8
  %_3 = load ptr, ptr %_1, align 8, !nonnull !2, !align !4, !noundef !2
  call void @llvm.memcpy.p0.p0.i64(ptr align 8 %_2, ptr align 8 %_3, i64 48, i1 false)
; call alloc::fmt::format::format_inner
  call void @_ZN5alloc3fmt6format12format_inner17hb8a7cc9ead64df92E(ptr sret(%"alloc::string::String") %0, ptr %_2)
  ret void
}

; alloc::str::<impl alloc::borrow::ToOwned for str>::to_owned
; Function Attrs: inlinehint nonlazybind uwtable
define internal void @"_ZN5alloc3str56_$LT$impl$u20$alloc..borrow..ToOwned$u20$for$u20$str$GT$8to_owned17h4b90106d4b08620fE"(ptr sret(%"alloc::string::String") %0, ptr align 1 %self.0, i64 %self.1) unnamed_addr #0 {
start:
  %1 = alloca { ptr, i64 }, align 8
  %bytes = alloca %"alloc::vec::Vec<u8>", align 8
  %2 = getelementptr inbounds { ptr, i64 }, ptr %1, i32 0, i32 0
  store ptr %self.0, ptr %2, align 8
  %3 = getelementptr inbounds { ptr, i64 }, ptr %1, i32 0, i32 1
  store i64 %self.1, ptr %3, align 8
  %4 = getelementptr inbounds { ptr, i64 }, ptr %1, i32 0, i32 0
  %_4.0 = load ptr, ptr %4, align 8, !nonnull !2, !align !3, !noundef !2
  %5 = getelementptr inbounds { ptr, i64 }, ptr %1, i32 0, i32 1
  %_4.1 = load i64, ptr %5, align 8, !noundef !2
; call alloc::slice::<impl alloc::borrow::ToOwned for [T]>::to_owned
  call void @"_ZN5alloc5slice64_$LT$impl$u20$alloc..borrow..ToOwned$u20$for$u20$$u5b$T$u5d$$GT$8to_owned17h816aa97eb3bb5617E"(ptr sret(%"alloc::vec::Vec<u8>") %bytes, ptr align 1 %_4.0, i64 %_4.1)
  call void @llvm.memcpy.p0.p0.i64(ptr align 8 %0, ptr align 8 %bytes, i64 24, i1 false)
  ret void
}

; alloc::alloc::Global::alloc_impl
; Function Attrs: inlinehint nonlazybind uwtable
define internal { ptr, i64 } @_ZN5alloc5alloc6Global10alloc_impl17hbfc95029f56be964E(ptr align 1 %self, i64 %0, i64 %1, i1 zeroext %zeroed) unnamed_addr #0 {
start:
  %2 = alloca ptr, align 8
  %3 = alloca i64, align 8
  %_83 = alloca { ptr, i64 }, align 8
  %_82 = alloca %"core::ptr::metadata::PtrRepr<[u8]>", align 8
  %_64 = alloca ptr, align 8
  %_63 = alloca ptr, align 8
  %_57 = alloca i64, align 8
  %_48 = alloca i64, align 8
  %_38 = alloca { ptr, i64 }, align 8
  %_37 = alloca %"core::ptr::metadata::PtrRepr<[u8]>", align 8
  %_25 = alloca i64, align 8
  %_21 = alloca { ptr, i64 }, align 8
  %self4 = alloca ptr, align 8
  %self3 = alloca ptr, align 8
  %_12 = alloca ptr, align 8
  %layout2 = alloca { i64, i64 }, align 8
  %layout1 = alloca { i64, i64 }, align 8
  %raw_ptr = alloca ptr, align 8
  %data = alloca ptr, align 8
  %_6 = alloca { ptr, i64 }, align 8
  %4 = alloca { ptr, i64 }, align 8
  %layout = alloca { i64, i64 }, align 8
  %5 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  store i64 %0, ptr %5, align 8
  %6 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  store i64 %1, ptr %6, align 8
  %size = load i64, ptr %layout, align 8, !noundef !2
  %7 = icmp eq i64 %size, 0
  br i1 %7, label %bb2, label %bb1

bb2:                                              ; preds = %start
  %8 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %self10 = load i64, ptr %8, align 8, !range !6, !noundef !2
  store i64 %self10, ptr %_25, align 8
  %_26 = load i64, ptr %_25, align 8, !range !6, !noundef !2
  %_27 = icmp uge i64 -9223372036854775808, %_26
  call void @llvm.assume(i1 %_27)
  %_28 = icmp ule i64 1, %_26
  call void @llvm.assume(i1 %_28)
  store i64 %_26, ptr %2, align 8
  %ptr11 = load ptr, ptr %2, align 8, !noundef !2
  store ptr %ptr11, ptr %data, align 8
  %_35 = load ptr, ptr %data, align 8, !noundef !2
  store ptr %_35, ptr %_38, align 8
  %9 = getelementptr inbounds { ptr, i64 }, ptr %_38, i32 0, i32 1
  store i64 0, ptr %9, align 8
  %10 = getelementptr inbounds { ptr, i64 }, ptr %_38, i32 0, i32 0
  %11 = load ptr, ptr %10, align 8, !noundef !2
  %12 = getelementptr inbounds { ptr, i64 }, ptr %_38, i32 0, i32 1
  %13 = load i64, ptr %12, align 8, !noundef !2
  %14 = getelementptr inbounds { ptr, i64 }, ptr %_37, i32 0, i32 0
  store ptr %11, ptr %14, align 8
  %15 = getelementptr inbounds { ptr, i64 }, ptr %_37, i32 0, i32 1
  store i64 %13, ptr %15, align 8
  %16 = getelementptr inbounds { ptr, i64 }, ptr %_37, i32 0, i32 0
  %ptr.012 = load ptr, ptr %16, align 8, !noundef !2
  %17 = getelementptr inbounds { ptr, i64 }, ptr %_37, i32 0, i32 1
  %ptr.113 = load i64, ptr %17, align 8, !noundef !2
  %18 = getelementptr inbounds { ptr, i64 }, ptr %_6, i32 0, i32 0
  store ptr %ptr.012, ptr %18, align 8
  %19 = getelementptr inbounds { ptr, i64 }, ptr %_6, i32 0, i32 1
  store i64 %ptr.113, ptr %19, align 8
  %20 = getelementptr inbounds { ptr, i64 }, ptr %_6, i32 0, i32 0
  %21 = load ptr, ptr %20, align 8, !nonnull !2, !noundef !2
  %22 = getelementptr inbounds { ptr, i64 }, ptr %_6, i32 0, i32 1
  %23 = load i64, ptr %22, align 8, !noundef !2
  %24 = getelementptr inbounds { ptr, i64 }, ptr %4, i32 0, i32 0
  store ptr %21, ptr %24, align 8
  %25 = getelementptr inbounds { ptr, i64 }, ptr %4, i32 0, i32 1
  store i64 %23, ptr %25, align 8
  br label %bb10

bb1:                                              ; preds = %start
  br i1 %zeroed, label %bb3, label %bb4

bb4:                                              ; preds = %bb1
  %26 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  %27 = load i64, ptr %26, align 8, !noundef !2
  %28 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %29 = load i64, ptr %28, align 8, !range !6, !noundef !2
  %30 = getelementptr inbounds { i64, i64 }, ptr %layout2, i32 0, i32 0
  store i64 %27, ptr %30, align 8
  %31 = getelementptr inbounds { i64, i64 }, ptr %layout2, i32 0, i32 1
  store i64 %29, ptr %31, align 8
  %_52 = load i64, ptr %layout2, align 8, !noundef !2
  %32 = getelementptr inbounds { i64, i64 }, ptr %layout2, i32 0, i32 1
  %self6 = load i64, ptr %32, align 8, !range !6, !noundef !2
  store i64 %self6, ptr %_57, align 8
  %_58 = load i64, ptr %_57, align 8, !range !6, !noundef !2
  %_59 = icmp uge i64 -9223372036854775808, %_58
  call void @llvm.assume(i1 %_59)
  %_60 = icmp ule i64 1, %_58
  call void @llvm.assume(i1 %_60)
  %33 = call ptr @__rust_alloc(i64 %_52, i64 %_58) #15
  store ptr %33, ptr %raw_ptr, align 8
  br label %bb5

bb3:                                              ; preds = %bb1
  %34 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  %35 = load i64, ptr %34, align 8, !noundef !2
  %36 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %37 = load i64, ptr %36, align 8, !range !6, !noundef !2
  %38 = getelementptr inbounds { i64, i64 }, ptr %layout1, i32 0, i32 0
  store i64 %35, ptr %38, align 8
  %39 = getelementptr inbounds { i64, i64 }, ptr %layout1, i32 0, i32 1
  store i64 %37, ptr %39, align 8
  %_43 = load i64, ptr %layout1, align 8, !noundef !2
  %40 = getelementptr inbounds { i64, i64 }, ptr %layout1, i32 0, i32 1
  %self5 = load i64, ptr %40, align 8, !range !6, !noundef !2
  store i64 %self5, ptr %_48, align 8
  %_49 = load i64, ptr %_48, align 8, !range !6, !noundef !2
  %_50 = icmp uge i64 -9223372036854775808, %_49
  call void @llvm.assume(i1 %_50)
  %_51 = icmp ule i64 1, %_49
  call void @llvm.assume(i1 %_51)
  %41 = call ptr @__rust_alloc_zeroed(i64 %_43, i64 %_49) #15
  store ptr %41, ptr %raw_ptr, align 8
  br label %bb5

bb5:                                              ; preds = %bb4, %bb3
  %ptr = load ptr, ptr %raw_ptr, align 8, !noundef !2
  store ptr %ptr, ptr %_64, align 8
  %ptr7 = load ptr, ptr %_64, align 8, !noundef !2
  store ptr %ptr7, ptr %3, align 8
  %_66 = load i64, ptr %3, align 8, !noundef !2
  %_62 = icmp eq i64 %_66, 0
  %_61 = xor i1 %_62, true
  br i1 %_61, label %bb14, label %bb15

bb15:                                             ; preds = %bb5
  store ptr null, ptr %self4, align 8
  br label %bb16

bb14:                                             ; preds = %bb5
  store ptr %ptr, ptr %_63, align 8
  %42 = load ptr, ptr %_63, align 8, !nonnull !2, !noundef !2
  store ptr %42, ptr %self4, align 8
  br label %bb16

bb16:                                             ; preds = %bb15, %bb14
  %43 = load ptr, ptr %self4, align 8, !noundef !2
  %44 = ptrtoint ptr %43 to i64
  %45 = icmp eq i64 %44, 0
  %_71 = select i1 %45, i64 0, i64 1
  %46 = icmp eq i64 %_71, 0
  br i1 %46, label %bb18, label %bb20

bb18:                                             ; preds = %bb16
  store ptr null, ptr %self3, align 8
  br label %bb21

bb20:                                             ; preds = %bb16
  %v = load ptr, ptr %self4, align 8, !nonnull !2, !noundef !2
  store ptr %v, ptr %self3, align 8
  br label %bb21

bb19:                                             ; No predecessors!
  unreachable

bb21:                                             ; preds = %bb18, %bb20
  %47 = load ptr, ptr %self3, align 8, !noundef !2
  %48 = ptrtoint ptr %47 to i64
  %49 = icmp eq i64 %48, 0
  %_74 = select i1 %49, i64 1, i64 0
  %50 = icmp eq i64 %_74, 0
  br i1 %50, label %bb24, label %bb22

bb24:                                             ; preds = %bb21
  %v8 = load ptr, ptr %self3, align 8, !nonnull !2, !noundef !2
  store ptr %v8, ptr %_12, align 8
  br label %bb6

bb22:                                             ; preds = %bb21
  store ptr null, ptr %_12, align 8
  br label %bb6

bb23:                                             ; No predecessors!
  unreachable

bb6:                                              ; preds = %bb24, %bb22
  %51 = load ptr, ptr %_12, align 8, !noundef !2
  %52 = ptrtoint ptr %51 to i64
  %53 = icmp eq i64 %52, 0
  %_17 = select i1 %53, i64 1, i64 0
  %54 = icmp eq i64 %_17, 0
  br i1 %54, label %bb7, label %bb9

bb7:                                              ; preds = %bb6
  %ptr9 = load ptr, ptr %_12, align 8, !nonnull !2, !noundef !2
  store ptr %ptr9, ptr %_83, align 8
  %55 = getelementptr inbounds { ptr, i64 }, ptr %_83, i32 0, i32 1
  store i64 %size, ptr %55, align 8
  %56 = getelementptr inbounds { ptr, i64 }, ptr %_83, i32 0, i32 0
  %57 = load ptr, ptr %56, align 8, !noundef !2
  %58 = getelementptr inbounds { ptr, i64 }, ptr %_83, i32 0, i32 1
  %59 = load i64, ptr %58, align 8, !noundef !2
  %60 = getelementptr inbounds { ptr, i64 }, ptr %_82, i32 0, i32 0
  store ptr %57, ptr %60, align 8
  %61 = getelementptr inbounds { ptr, i64 }, ptr %_82, i32 0, i32 1
  store i64 %59, ptr %61, align 8
  %62 = getelementptr inbounds { ptr, i64 }, ptr %_82, i32 0, i32 0
  %ptr.0 = load ptr, ptr %62, align 8, !noundef !2
  %63 = getelementptr inbounds { ptr, i64 }, ptr %_82, i32 0, i32 1
  %ptr.1 = load i64, ptr %63, align 8, !noundef !2
  %64 = getelementptr inbounds { ptr, i64 }, ptr %_21, i32 0, i32 0
  store ptr %ptr.0, ptr %64, align 8
  %65 = getelementptr inbounds { ptr, i64 }, ptr %_21, i32 0, i32 1
  store i64 %ptr.1, ptr %65, align 8
  %66 = getelementptr inbounds { ptr, i64 }, ptr %_21, i32 0, i32 0
  %67 = load ptr, ptr %66, align 8, !nonnull !2, !noundef !2
  %68 = getelementptr inbounds { ptr, i64 }, ptr %_21, i32 0, i32 1
  %69 = load i64, ptr %68, align 8, !noundef !2
  %70 = getelementptr inbounds { ptr, i64 }, ptr %4, i32 0, i32 0
  store ptr %67, ptr %70, align 8
  %71 = getelementptr inbounds { ptr, i64 }, ptr %4, i32 0, i32 1
  store i64 %69, ptr %71, align 8
  br label %bb10

bb9:                                              ; preds = %bb6
  store ptr null, ptr %4, align 8
  br label %bb10

bb8:                                              ; No predecessors!
  unreachable

bb10:                                             ; preds = %bb2, %bb7, %bb9
  %72 = getelementptr inbounds { ptr, i64 }, ptr %4, i32 0, i32 0
  %73 = load ptr, ptr %72, align 8, !noundef !2
  %74 = getelementptr inbounds { ptr, i64 }, ptr %4, i32 0, i32 1
  %75 = load i64, ptr %74, align 8
  %76 = insertvalue { ptr, i64 } undef, ptr %73, 0
  %77 = insertvalue { ptr, i64 } %76, i64 %75, 1
  ret { ptr, i64 } %77
}

; alloc::slice::<impl alloc::borrow::ToOwned for [T]>::to_owned
; Function Attrs: nonlazybind uwtable
define void @"_ZN5alloc5slice64_$LT$impl$u20$alloc..borrow..ToOwned$u20$for$u20$$u5b$T$u5d$$GT$8to_owned17h816aa97eb3bb5617E"(ptr sret(%"alloc::vec::Vec<u8>") %0, ptr align 1 %self.0, i64 %self.1) unnamed_addr #1 {
start:
; call <T as alloc::slice::hack::ConvertVec>::to_vec
  call void @"_ZN52_$LT$T$u20$as$u20$alloc..slice..hack..ConvertVec$GT$6to_vec17h9128de7e47161cabE"(ptr sret(%"alloc::vec::Vec<u8>") %0, ptr align 1 %self.0, i64 %self.1)
  ret void
}

; alloc::raw_vec::RawVec<T,A>::allocate_in
; Function Attrs: nonlazybind uwtable
define { i64, ptr } @"_ZN5alloc7raw_vec19RawVec$LT$T$C$A$GT$11allocate_in17h0e1e1f61f43e6e42E"(i64 %capacity, i1 zeroext %0) unnamed_addr #1 personality ptr @rust_eh_personality {
start:
  %1 = alloca i64, align 8
  %2 = alloca { ptr, i32 }, align 8
  %_41 = alloca ptr, align 8
  %_28 = alloca i8, align 1
  %self = alloca ptr, align 8
  %_24 = alloca ptr, align 8
  %result = alloca { ptr, i64 }, align 8
  %_12 = alloca { i64, i64 }, align 8
  %_8 = alloca { i64, i64 }, align 8
  %_4 = alloca i8, align 1
  %3 = alloca { i64, ptr }, align 8
  %alloc = alloca %"alloc::alloc::Global", align 1
  %init = alloca i8, align 1
  %4 = zext i1 %0 to i8
  store i8 %4, ptr %init, align 1
  store i8 1, ptr %_28, align 1
  br i1 false, label %bb1, label %bb2

bb2:                                              ; preds = %start
  %_5 = icmp eq i64 %capacity, 0
  %5 = zext i1 %_5 to i8
  store i8 %5, ptr %_4, align 1
  br label %bb3

bb1:                                              ; preds = %start
  store i8 1, ptr %_4, align 1
  br label %bb3

bb3:                                              ; preds = %bb2, %bb1
  %6 = load i8, ptr %_4, align 1, !range !5, !noundef !2
  %7 = trunc i8 %6 to i1
  br i1 %7, label %bb4, label %bb6

bb6:                                              ; preds = %bb3
  store i64 1, ptr %1, align 8
  %_30 = load i64, ptr %1, align 8, !range !6, !noundef !2
  br label %bb27

bb4:                                              ; preds = %bb3
  store i8 0, ptr %_28, align 1
; invoke alloc::raw_vec::RawVec<T,A>::new_in
  %8 = invoke { i64, ptr } @"_ZN5alloc7raw_vec19RawVec$LT$T$C$A$GT$6new_in17h1b9cf0c96069bf73E"()
          to label %bb5 unwind label %cleanup

bb25:                                             ; preds = %cleanup
  %9 = load i8, ptr %_28, align 1, !range !5, !noundef !2
  %10 = trunc i8 %9 to i1
  br i1 %10, label %bb24, label %bb23

cleanup:                                          ; preds = %bb19, %bb15, %bb13, %bb10, %bb7, %bb27, %bb4
  %11 = landingpad { ptr, i32 }
          cleanup
  %12 = extractvalue { ptr, i32 } %11, 0
  %13 = extractvalue { ptr, i32 } %11, 1
  %14 = getelementptr inbounds { ptr, i32 }, ptr %2, i32 0, i32 0
  store ptr %12, ptr %14, align 8
  %15 = getelementptr inbounds { ptr, i32 }, ptr %2, i32 0, i32 1
  store i32 %13, ptr %15, align 8
  br label %bb25

bb5:                                              ; preds = %bb4
  store { i64, ptr } %8, ptr %3, align 8
  br label %bb22

bb22:                                             ; preds = %bb21, %bb5
  %16 = getelementptr inbounds { i64, ptr }, ptr %3, i32 0, i32 0
  %17 = load i64, ptr %16, align 8, !noundef !2
  %18 = getelementptr inbounds { i64, ptr }, ptr %3, i32 0, i32 1
  %19 = load ptr, ptr %18, align 8, !nonnull !2, !noundef !2
  %20 = insertvalue { i64, ptr } undef, i64 %17, 0
  %21 = insertvalue { i64, ptr } %20, ptr %19, 1
  ret { i64, ptr } %21

bb27:                                             ; preds = %bb6
; invoke core::alloc::layout::Layout::array::inner
  %22 = invoke { i64, i64 } @_ZN4core5alloc6layout6Layout5array5inner17h286ae9a02469cf04E(i64 1, i64 %_30, i64 %capacity)
          to label %bb26 unwind label %cleanup

bb26:                                             ; preds = %bb27
  store { i64, i64 } %22, ptr %_8, align 8
  %23 = getelementptr inbounds { i64, i64 }, ptr %_8, i32 0, i32 1
  %24 = load i64, ptr %23, align 8, !range !7, !noundef !2
  %25 = icmp eq i64 %24, 0
  %_9 = select i1 %25, i64 1, i64 0
  %26 = icmp eq i64 %_9, 0
  br i1 %26, label %bb9, label %bb7

bb9:                                              ; preds = %bb26
  %27 = getelementptr inbounds { i64, i64 }, ptr %_8, i32 0, i32 0
  %layout.0 = load i64, ptr %27, align 8, !noundef !2
  %28 = getelementptr inbounds { i64, i64 }, ptr %_8, i32 0, i32 1
  %layout.1 = load i64, ptr %28, align 8, !range !6, !noundef !2
  %29 = getelementptr inbounds { i64, i64 }, ptr %_12, i32 0, i32 1
  store i64 -9223372036854775807, ptr %29, align 8
  %30 = getelementptr inbounds { i64, i64 }, ptr %_12, i32 0, i32 1
  %31 = load i64, ptr %30, align 8, !range !8, !noundef !2
  %32 = icmp eq i64 %31, -9223372036854775807
  %_15 = select i1 %32, i64 0, i64 1
  %33 = icmp eq i64 %_15, 0
  br i1 %33, label %bb12, label %bb10

bb7:                                              ; preds = %bb26
; invoke alloc::raw_vec::capacity_overflow
  invoke void @_ZN5alloc7raw_vec17capacity_overflow17h564475d43ac0e37cE() #12
          to label %unreachable unwind label %cleanup

bb8:                                              ; No predecessors!
  unreachable

unreachable:                                      ; preds = %bb19, %bb10, %bb7
  unreachable

bb12:                                             ; preds = %bb9
  %34 = load i8, ptr %init, align 1, !range !5, !noundef !2
  %35 = trunc i8 %34 to i1
  %_18 = zext i1 %35 to i64
  %36 = icmp eq i64 %_18, 0
  br i1 %36, label %bb15, label %bb13

bb10:                                             ; preds = %bb9
; invoke alloc::raw_vec::capacity_overflow
  invoke void @_ZN5alloc7raw_vec17capacity_overflow17h564475d43ac0e37cE() #12
          to label %unreachable unwind label %cleanup

bb11:                                             ; No predecessors!
  unreachable

bb15:                                             ; preds = %bb12
; invoke <alloc::alloc::Global as core::alloc::Allocator>::allocate
  %37 = invoke { ptr, i64 } @"_ZN63_$LT$alloc..alloc..Global$u20$as$u20$core..alloc..Allocator$GT$8allocate17h6a5912a9cec350eaE"(ptr align 1 %alloc, i64 %layout.0, i64 %layout.1)
          to label %bb16 unwind label %cleanup

bb13:                                             ; preds = %bb12
; invoke <alloc::alloc::Global as core::alloc::Allocator>::allocate_zeroed
  %38 = invoke { ptr, i64 } @"_ZN63_$LT$alloc..alloc..Global$u20$as$u20$core..alloc..Allocator$GT$15allocate_zeroed17h0612da8cbf366602E"(ptr align 1 %alloc, i64 %layout.0, i64 %layout.1)
          to label %bb17 unwind label %cleanup

bb14:                                             ; No predecessors!
  unreachable

bb17:                                             ; preds = %bb13
  store { ptr, i64 } %38, ptr %result, align 8
  br label %bb18

bb18:                                             ; preds = %bb16, %bb17
  %39 = load ptr, ptr %result, align 8, !noundef !2
  %40 = ptrtoint ptr %39 to i64
  %41 = icmp eq i64 %40, 0
  %_21 = select i1 %41, i64 1, i64 0
  %42 = icmp eq i64 %_21, 0
  br i1 %42, label %bb21, label %bb19

bb16:                                             ; preds = %bb15
  store { ptr, i64 } %37, ptr %result, align 8
  br label %bb18

bb21:                                             ; preds = %bb18
  %43 = getelementptr inbounds { ptr, i64 }, ptr %result, i32 0, i32 0
  %ptr.0 = load ptr, ptr %43, align 8, !nonnull !2, !noundef !2
  %44 = getelementptr inbounds { ptr, i64 }, ptr %result, i32 0, i32 1
  %ptr.1 = load i64, ptr %44, align 8, !noundef !2
  store ptr %ptr.0, ptr %self, align 8
  %_40 = load ptr, ptr %self, align 8, !noundef !2
  store ptr %_40, ptr %_41, align 8
  %45 = load ptr, ptr %_41, align 8, !nonnull !2, !noundef !2
  store ptr %45, ptr %_24, align 8
  %46 = load ptr, ptr %_24, align 8, !nonnull !2, !noundef !2
  %47 = getelementptr inbounds { i64, ptr }, ptr %3, i32 0, i32 1
  store ptr %46, ptr %47, align 8
  store i64 %capacity, ptr %3, align 8
  br label %bb22

bb19:                                             ; preds = %bb18
; invoke alloc::alloc::handle_alloc_error
  invoke void @_ZN5alloc5alloc18handle_alloc_error17h07edb87aaab24c34E(i64 %layout.0, i64 %layout.1) #12
          to label %unreachable unwind label %cleanup

bb20:                                             ; No predecessors!
  unreachable

bb23:                                             ; preds = %bb24, %bb25
  %48 = load ptr, ptr %2, align 8, !noundef !2
  %49 = getelementptr inbounds { ptr, i32 }, ptr %2, i32 0, i32 1
  %50 = load i32, ptr %49, align 8, !noundef !2
  %51 = insertvalue { ptr, i32 } undef, ptr %48, 0
  %52 = insertvalue { ptr, i32 } %51, i32 %50, 1
  resume { ptr, i32 } %52

bb24:                                             ; preds = %bb25
  br label %bb23
}

; alloc::raw_vec::RawVec<T,A>::current_memory
; Function Attrs: nonlazybind uwtable
define void @"_ZN5alloc7raw_vec19RawVec$LT$T$C$A$GT$14current_memory17h82acd859da8049a5E"(ptr sret(%"core::option::Option<(core::ptr::non_null::NonNull<u8>, core::alloc::layout::Layout)>") %0, ptr align 8 %self) unnamed_addr #1 {
start:
  %1 = alloca i64, align 8
  %2 = alloca i64, align 8
  %pointer = alloca ptr, align 8
  %_13 = alloca ptr, align 8
  %_11 = alloca { ptr, { i64, i64 } }, align 8
  %layout = alloca { i64, i64 }, align 8
  %_2 = alloca i8, align 1
  br i1 false, label %bb1, label %bb2

bb2:                                              ; preds = %start
  %_4 = load i64, ptr %self, align 8, !noundef !2
  %_3 = icmp eq i64 %_4, 0
  %3 = zext i1 %_3 to i8
  store i8 %3, ptr %_2, align 1
  br label %bb3

bb1:                                              ; preds = %start
  store i8 1, ptr %_2, align 1
  br label %bb3

bb3:                                              ; preds = %bb2, %bb1
  %4 = load i8, ptr %_2, align 1, !range !5, !noundef !2
  %5 = trunc i8 %4 to i1
  br i1 %5, label %bb4, label %bb5

bb5:                                              ; preds = %bb3
  %rhs = load i64, ptr %self, align 8, !noundef !2
  %6 = mul nuw i64 1, %rhs
  store i64 %6, ptr %2, align 8
  %size = load i64, ptr %2, align 8, !noundef !2
  store i64 1, ptr %1, align 8
  %_15 = load i64, ptr %1, align 8, !range !6, !noundef !2
  store i64 %size, ptr %layout, align 8
  %7 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  store i64 %_15, ptr %7, align 8
  %8 = getelementptr inbounds { i64, ptr }, ptr %self, i32 0, i32 1
  %self1 = load ptr, ptr %8, align 8, !nonnull !2, !noundef !2
  store ptr %self1, ptr %pointer, align 8
  %9 = load ptr, ptr %pointer, align 8, !nonnull !2, !noundef !2
  store ptr %9, ptr %_13, align 8
  %10 = load ptr, ptr %_13, align 8, !nonnull !2, !noundef !2
; call <T as core::convert::Into<U>>::into
  %_12 = call ptr @"_ZN50_$LT$T$u20$as$u20$core..convert..Into$LT$U$GT$$GT$4into17h6d20ebae4fbb1ff4E"(ptr %10)
  store ptr %_12, ptr %_11, align 8
  %11 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  %12 = load i64, ptr %11, align 8, !noundef !2
  %13 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %14 = load i64, ptr %13, align 8, !range !6, !noundef !2
  %15 = getelementptr inbounds { ptr, { i64, i64 } }, ptr %_11, i32 0, i32 1
  %16 = getelementptr inbounds { i64, i64 }, ptr %15, i32 0, i32 0
  store i64 %12, ptr %16, align 8
  %17 = getelementptr inbounds { i64, i64 }, ptr %15, i32 0, i32 1
  store i64 %14, ptr %17, align 8
  call void @llvm.memcpy.p0.p0.i64(ptr align 8 %0, ptr align 8 %_11, i64 24, i1 false)
  br label %bb7

bb4:                                              ; preds = %bb3
  %18 = getelementptr inbounds %"core::option::Option<(core::ptr::non_null::NonNull<u8>, core::alloc::layout::Layout)>", ptr %0, i32 0, i32 1
  store i64 0, ptr %18, align 8
  br label %bb7

bb7:                                              ; preds = %bb5, %bb4
  ret void
}

; alloc::raw_vec::RawVec<T,A>::new_in
; Function Attrs: nonlazybind uwtable
define { i64, ptr } @"_ZN5alloc7raw_vec19RawVec$LT$T$C$A$GT$6new_in17h1b9cf0c96069bf73E"() unnamed_addr #1 personality ptr @rust_eh_personality {
start:
  %0 = alloca { ptr, i32 }, align 8
  %1 = alloca ptr, align 8
  %pointer = alloca ptr, align 8
  %_2 = alloca ptr, align 8
  %2 = alloca { i64, ptr }, align 8
  store i64 1, ptr %1, align 8
  %ptr = load ptr, ptr %1, align 8, !noundef !2
  br label %bb3

bb3:                                              ; preds = %start
  store ptr %ptr, ptr %pointer, align 8
  %3 = load ptr, ptr %pointer, align 8, !nonnull !2, !noundef !2
  store ptr %3, ptr %_2, align 8
  %4 = load ptr, ptr %_2, align 8, !nonnull !2, !noundef !2
  %5 = getelementptr inbounds { i64, ptr }, ptr %2, i32 0, i32 1
  store ptr %4, ptr %5, align 8
  store i64 0, ptr %2, align 8
  %6 = getelementptr inbounds { i64, ptr }, ptr %2, i32 0, i32 0
  %7 = load i64, ptr %6, align 8, !noundef !2
  %8 = getelementptr inbounds { i64, ptr }, ptr %2, i32 0, i32 1
  %9 = load ptr, ptr %8, align 8, !nonnull !2, !noundef !2
  %10 = insertvalue { i64, ptr } undef, i64 %7, 0
  %11 = insertvalue { i64, ptr } %10, ptr %9, 1
  ret { i64, ptr } %11

bb1:                                              ; No predecessors!
  %12 = load ptr, ptr %0, align 8, !noundef !2
  %13 = getelementptr inbounds { ptr, i32 }, ptr %0, i32 0, i32 1
  %14 = load i32, ptr %13, align 8, !noundef !2
  %15 = insertvalue { ptr, i32 } undef, ptr %12, 0
  %16 = insertvalue { ptr, i32 } %15, i32 %14, 1
  resume { ptr, i32 } %16
}

; <alloc::alloc::Global as core::alloc::Allocator>::deallocate
; Function Attrs: inlinehint nonlazybind uwtable
define internal void @"_ZN63_$LT$alloc..alloc..Global$u20$as$u20$core..alloc..Allocator$GT$10deallocate17h10fad0c1701e9564E"(ptr align 1 %self, ptr %ptr, i64 %0, i64 %1) unnamed_addr #0 {
start:
  %_14 = alloca i64, align 8
  %layout1 = alloca { i64, i64 }, align 8
  %layout = alloca { i64, i64 }, align 8
  %2 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  store i64 %0, ptr %2, align 8
  %3 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  store i64 %1, ptr %3, align 8
  %_4 = load i64, ptr %layout, align 8, !noundef !2
  %4 = icmp eq i64 %_4, 0
  br i1 %4, label %bb2, label %bb1

bb2:                                              ; preds = %start
  br label %bb3

bb1:                                              ; preds = %start
  %5 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  %6 = load i64, ptr %5, align 8, !noundef !2
  %7 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %8 = load i64, ptr %7, align 8, !range !6, !noundef !2
  %9 = getelementptr inbounds { i64, i64 }, ptr %layout1, i32 0, i32 0
  store i64 %6, ptr %9, align 8
  %10 = getelementptr inbounds { i64, i64 }, ptr %layout1, i32 0, i32 1
  store i64 %8, ptr %10, align 8
  %_9 = load i64, ptr %layout1, align 8, !noundef !2
  %11 = getelementptr inbounds { i64, i64 }, ptr %layout1, i32 0, i32 1
  %self2 = load i64, ptr %11, align 8, !range !6, !noundef !2
  store i64 %self2, ptr %_14, align 8
  %_15 = load i64, ptr %_14, align 8, !range !6, !noundef !2
  %_16 = icmp uge i64 -9223372036854775808, %_15
  call void @llvm.assume(i1 %_16)
  %_17 = icmp ule i64 1, %_15
  call void @llvm.assume(i1 %_17)
  call void @__rust_dealloc(ptr %ptr, i64 %_9, i64 %_15) #15
  br label %bb3

bb3:                                              ; preds = %bb2, %bb1
  ret void
}

; <alloc::alloc::Global as core::alloc::Allocator>::allocate_zeroed
; Function Attrs: inlinehint nonlazybind uwtable
define internal { ptr, i64 } @"_ZN63_$LT$alloc..alloc..Global$u20$as$u20$core..alloc..Allocator$GT$15allocate_zeroed17h0612da8cbf366602E"(ptr align 1 %self, i64 %layout.0, i64 %layout.1) unnamed_addr #0 {
start:
; call alloc::alloc::Global::alloc_impl
  %0 = call { ptr, i64 } @_ZN5alloc5alloc6Global10alloc_impl17hbfc95029f56be964E(ptr align 1 %self, i64 %layout.0, i64 %layout.1, i1 zeroext true)
  %1 = extractvalue { ptr, i64 } %0, 0
  %2 = extractvalue { ptr, i64 } %0, 1
  %3 = insertvalue { ptr, i64 } undef, ptr %1, 0
  %4 = insertvalue { ptr, i64 } %3, i64 %2, 1
  ret { ptr, i64 } %4
}

; <alloc::alloc::Global as core::alloc::Allocator>::allocate
; Function Attrs: inlinehint nonlazybind uwtable
define internal { ptr, i64 } @"_ZN63_$LT$alloc..alloc..Global$u20$as$u20$core..alloc..Allocator$GT$8allocate17h6a5912a9cec350eaE"(ptr align 1 %self, i64 %layout.0, i64 %layout.1) unnamed_addr #0 {
start:
; call alloc::alloc::Global::alloc_impl
  %0 = call { ptr, i64 } @_ZN5alloc5alloc6Global10alloc_impl17hbfc95029f56be964E(ptr align 1 %self, i64 %layout.0, i64 %layout.1, i1 zeroext false)
  %1 = extractvalue { ptr, i64 } %0, 0
  %2 = extractvalue { ptr, i64 } %0, 1
  %3 = insertvalue { ptr, i64 } undef, ptr %1, 0
  %4 = insertvalue { ptr, i64 } %3, i64 %2, 1
  ret { ptr, i64 } %4
}

; <alloc::vec::Vec<T,A> as core::ops::drop::Drop>::drop
; Function Attrs: nonlazybind uwtable
define void @"_ZN70_$LT$alloc..vec..Vec$LT$T$C$A$GT$$u20$as$u20$core..ops..drop..Drop$GT$4drop17h4ec93900252c0b8dE"(ptr align 8 %self) unnamed_addr #1 {
start:
  %0 = alloca i64, align 8
  %_18 = alloca { ptr, i64 }, align 8
  %_17 = alloca %"core::ptr::metadata::PtrRepr<[u8]>", align 8
  %_11 = alloca ptr, align 8
  %1 = getelementptr inbounds { i64, ptr }, ptr %self, i32 0, i32 1
  %self1 = load ptr, ptr %1, align 8, !nonnull !2, !noundef !2
  store ptr %self1, ptr %_11, align 8
  %ptr = load ptr, ptr %_11, align 8, !noundef !2
  store ptr %ptr, ptr %0, align 8
  %_14 = load i64, ptr %0, align 8, !noundef !2
  %_7 = icmp eq i64 %_14, 0
  %_6 = xor i1 %_7, true
  call void @llvm.assume(i1 %_6)
  %2 = getelementptr inbounds %"alloc::vec::Vec<u8>", ptr %self, i32 0, i32 1
  %len = load i64, ptr %2, align 8, !noundef !2
  store ptr %self1, ptr %_18, align 8
  %3 = getelementptr inbounds { ptr, i64 }, ptr %_18, i32 0, i32 1
  store i64 %len, ptr %3, align 8
  %4 = getelementptr inbounds { ptr, i64 }, ptr %_18, i32 0, i32 0
  %5 = load ptr, ptr %4, align 8, !noundef !2
  %6 = getelementptr inbounds { ptr, i64 }, ptr %_18, i32 0, i32 1
  %7 = load i64, ptr %6, align 8, !noundef !2
  %8 = getelementptr inbounds { ptr, i64 }, ptr %_17, i32 0, i32 0
  store ptr %5, ptr %8, align 8
  %9 = getelementptr inbounds { ptr, i64 }, ptr %_17, i32 0, i32 1
  store i64 %7, ptr %9, align 8
  %10 = getelementptr inbounds { ptr, i64 }, ptr %_17, i32 0, i32 0
  %_2.0 = load ptr, ptr %10, align 8, !noundef !2
  %11 = getelementptr inbounds { ptr, i64 }, ptr %_17, i32 0, i32 1
  %_2.1 = load i64, ptr %11, align 8, !noundef !2
  ret void
}

; <alloc::raw_vec::RawVec<T,A> as core::ops::drop::Drop>::drop
; Function Attrs: nonlazybind uwtable
define void @"_ZN77_$LT$alloc..raw_vec..RawVec$LT$T$C$A$GT$$u20$as$u20$core..ops..drop..Drop$GT$4drop17h6334db8ef1f3288eE"(ptr align 8 %self) unnamed_addr #1 {
start:
  %_2 = alloca %"core::option::Option<(core::ptr::non_null::NonNull<u8>, core::alloc::layout::Layout)>", align 8
; call alloc::raw_vec::RawVec<T,A>::current_memory
  call void @"_ZN5alloc7raw_vec19RawVec$LT$T$C$A$GT$14current_memory17h82acd859da8049a5E"(ptr sret(%"core::option::Option<(core::ptr::non_null::NonNull<u8>, core::alloc::layout::Layout)>") %_2, ptr align 8 %self)
  %0 = getelementptr inbounds %"core::option::Option<(core::ptr::non_null::NonNull<u8>, core::alloc::layout::Layout)>", ptr %_2, i32 0, i32 1
  %1 = load i64, ptr %0, align 8, !range !7, !noundef !2
  %2 = icmp eq i64 %1, 0
  %_4 = select i1 %2, i64 0, i64 1
  %3 = icmp eq i64 %_4, 1
  br i1 %3, label %bb2, label %bb4

bb2:                                              ; preds = %start
  %ptr = load ptr, ptr %_2, align 8, !nonnull !2, !noundef !2
  %4 = getelementptr inbounds { ptr, { i64, i64 } }, ptr %_2, i32 0, i32 1
  %5 = getelementptr inbounds { i64, i64 }, ptr %4, i32 0, i32 0
  %layout.0 = load i64, ptr %5, align 8, !noundef !2
  %6 = getelementptr inbounds { i64, i64 }, ptr %4, i32 0, i32 1
  %layout.1 = load i64, ptr %6, align 8, !range !6, !noundef !2
; call <alloc::alloc::Global as core::alloc::Allocator>::deallocate
  call void @"_ZN63_$LT$alloc..alloc..Global$u20$as$u20$core..alloc..Allocator$GT$10deallocate17h10fad0c1701e9564E"(ptr align 1 %self, ptr %ptr, i64 %layout.0, i64 %layout.1)
  br label %bb4

bb4:                                              ; preds = %bb2, %start
  ret void
}

; probe1::probe
; Function Attrs: nonlazybind uwtable
define void @_ZN6probe15probe17h8ec139a6c12e9110E() unnamed_addr #1 {
start:
  %_7 = alloca [1 x { ptr, ptr }], align 8
  %_3 = alloca %"core::fmt::Arguments<'_>", align 8
  %res = alloca %"alloc::string::String", align 8
  %_1 = alloca %"alloc::string::String", align 8
; call core::fmt::ArgumentV1::new_lower_exp
  %0 = call { ptr, ptr } @_ZN4core3fmt10ArgumentV113new_lower_exp17h8ac11525faa56e76E(ptr align 8 @alloc_3d6223e1ee89533f014a7f6f8d8992fe)
  %_8.0 = extractvalue { ptr, ptr } %0, 0
  %_8.1 = extractvalue { ptr, ptr } %0, 1
  %1 = getelementptr inbounds [1 x { ptr, ptr }], ptr %_7, i64 0, i64 0
  %2 = getelementptr inbounds { ptr, ptr }, ptr %1, i32 0, i32 0
  store ptr %_8.0, ptr %2, align 8
  %3 = getelementptr inbounds { ptr, ptr }, ptr %1, i32 0, i32 1
  store ptr %_8.1, ptr %3, align 8
; call core::fmt::Arguments::new_v1
  call void @_ZN4core3fmt9Arguments6new_v117hb0661b8fc4ebe3c8E(ptr sret(%"core::fmt::Arguments<'_>") %_3, ptr align 8 @alloc_997d7ac396f89f2a981093fc6d33b686, i64 1, ptr align 8 %_7, i64 1)
; call alloc::fmt::format
  call void @_ZN5alloc3fmt6format17hf71f9424c95d3754E(ptr sret(%"alloc::string::String") %res, ptr %_3)
  call void @llvm.memcpy.p0.p0.i64(ptr align 8 %_1, ptr align 8 %res, i64 24, i1 false)
; call core::ptr::drop_in_place<alloc::string::String>
  call void @"_ZN4core3ptr42drop_in_place$LT$alloc..string..String$GT$17hb4af148fd5621c75E"(ptr %_1)
  ret void
}

; core::fmt::num::imp::<impl core::fmt::LowerExp for isize>::fmt
; Function Attrs: nonlazybind uwtable
declare zeroext i1 @"_ZN4core3fmt3num3imp55_$LT$impl$u20$core..fmt..LowerExp$u20$for$u20$isize$GT$3fmt17h02e9c410ed26b14bE"(ptr align 8, ptr align 8) unnamed_addr #1

; core::panicking::panic_fmt
; Function Attrs: cold noinline noreturn nonlazybind uwtable
declare void @_ZN4core9panicking9panic_fmt17hf33a1475b4dc5c3eE(ptr, ptr align 8) unnamed_addr #2

; Function Attrs: nonlazybind uwtable
declare i32 @rust_eh_personality(i32, i32, i64, ptr, ptr) unnamed_addr #1

; core::panicking::panic_cannot_unwind
; Function Attrs: cold noinline noreturn nounwind nonlazybind uwtable
declare void @_ZN4core9panicking19panic_cannot_unwind17hd123d9c71473dcdaE() unnamed_addr #3

; Function Attrs: inaccessiblememonly nocallback nofree nosync nounwind willreturn
declare void @llvm.assume(i1 noundef) #4

; Function Attrs: nocallback nofree nosync nounwind readnone willreturn
declare i1 @llvm.expect.i1(i1, i1) #5

; core::panicking::panic
; Function Attrs: cold noinline noreturn nonlazybind uwtable
declare void @_ZN4core9panicking5panic17h9533b2fee90b999eE(ptr align 1, i64, ptr align 8) unnamed_addr #2

; Function Attrs: argmemonly nocallback nofree nounwind willreturn
declare void @llvm.memcpy.p0.p0.i64(ptr noalias nocapture writeonly, ptr noalias nocapture readonly, i64, i1 immarg) #6

; alloc::fmt::format::format_inner
; Function Attrs: nonlazybind uwtable
declare void @_ZN5alloc3fmt6format12format_inner17hb8a7cc9ead64df92E(ptr sret(%"alloc::string::String"), ptr) unnamed_addr #1

; Function Attrs: nounwind nonlazybind allockind("alloc,zeroed,aligned") allocsize(0) uwtable
declare noalias ptr @__rust_alloc_zeroed(i64, i64 allocalign) unnamed_addr #7

; Function Attrs: nounwind nonlazybind allockind("alloc,uninitialized,aligned") allocsize(0) uwtable
declare noalias ptr @__rust_alloc(i64, i64 allocalign) unnamed_addr #8

; alloc::raw_vec::capacity_overflow
; Function Attrs: noreturn nonlazybind uwtable
declare void @_ZN5alloc7raw_vec17capacity_overflow17h564475d43ac0e37cE() unnamed_addr #9

; alloc::alloc::handle_alloc_error
; Function Attrs: cold noreturn nonlazybind uwtable
declare void @_ZN5alloc5alloc18handle_alloc_error17h07edb87aaab24c34E(i64, i64) unnamed_addr #10

; Function Attrs: nounwind nonlazybind allockind("free") uwtable
declare void @__rust_dealloc(ptr allocptr, i64, i64) unnamed_addr #11

attributes #0 = { inlinehint nonlazybind uwtable "probe-stack"="__rust_probestack" "target-cpu"="x86-64" }
attributes #1 = { nonlazybind uwtable "probe-stack"="__rust_probestack" "target-cpu"="x86-64" }
attributes #2 = { cold noinline noreturn nonlazybind uwtable "probe-stack"="__rust_probestack" "target-cpu"="x86-64" }
attributes #3 = { cold noinline noreturn nounwind nonlazybind uwtable "probe-stack"="__rust_probestack" "target-cpu"="x86-64" }
attributes #4 = { inaccessiblememonly nocallback nofree nosync nounwind willreturn }
attributes #5 = { nocallback nofree nosync nounwind readnone willreturn }
attributes #6 = { argmemonly nocallback nofree nounwind willreturn }
attributes #7 = { nounwind nonlazybind allockind("alloc,zeroed,aligned") allocsize(0) uwtable "alloc-family"="__rust_alloc" "probe-stack"="__rust_probestack" "target-cpu"="x86-64" }
attributes #8 = { nounwind nonlazybind allockind("alloc,uninitialized,aligned") allocsize(0) uwtable "alloc-family"="__rust_alloc" "probe-stack"="__rust_probestack" "target-cpu"="x86-64" }
attributes #9 = { noreturn nonlazybind uwtable "probe-stack"="__rust_probestack" "target-cpu"="x86-64" }
attributes #10 = { cold noreturn nonlazybind uwtable "probe-stack"="__rust_probestack" "target-cpu"="x86-64" }
attributes #11 = { nounwind nonlazybind allockind("free") uwtable "alloc-family"="__rust_alloc" "probe-stack"="__rust_probestack" "target-cpu"="x86-64" }
attributes #12 = { noreturn }
attributes #13 = { noinline }
attributes #14 = { noinline noreturn nounwind }
attributes #15 = { nounwind }

!llvm.module.flags = !{!0, !1}

!0 = !{i32 7, !"PIC Level", i32 2}
!1 = !{i32 2, !"RtLibUseGOT", i32 1}
!2 = !{}
!3 = !{i64 1}
!4 = !{i64 8}
!5 = !{i8 0, i8 2}
!6 = !{i64 1, i64 -9223372036854775807}
!7 = !{i64 0, i64 -9223372036854775807}
!8 = !{i64 0, i64 -9223372036854775806}
