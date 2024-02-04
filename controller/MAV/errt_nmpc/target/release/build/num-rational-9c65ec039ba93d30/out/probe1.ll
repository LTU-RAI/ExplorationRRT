; ModuleID = 'probe1.e5c80b9f5a0efe0a-cgu.0'
source_filename = "probe1.e5c80b9f5a0efe0a-cgu.0"
target datalayout = "e-m:e-p270:32:32-p271:32:32-p272:64:64-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

%"core::fmt::Arguments<'_>" = type { { ptr, i64 }, { ptr, i64 }, { ptr, i64 } }
%"alloc::string::String" = type { %"alloc::vec::Vec<u8>" }
%"alloc::vec::Vec<u8>" = type { { ptr, i64 }, i64 }
%"core::ptr::metadata::PtrRepr<[u8]>" = type { [2 x i64] }
%"alloc::alloc::Global" = type {}
%"core::option::Option<(core::ptr::non_null::NonNull<u8>, core::alloc::layout::Layout)>" = type { [1 x i64], i64, [1 x i64] }

@alloc_91c7fa63c3cfeaa3c795652d5cf060e4 = private unnamed_addr constant <{ [12 x i8] }> <{ [12 x i8] c"invalid args" }>, align 1
@alloc_560206a49c61adca6f3f0639a12632eb = private unnamed_addr constant <{ ptr, [8 x i8] }> <{ ptr @alloc_91c7fa63c3cfeaa3c795652d5cf060e4, [8 x i8] c"\0C\00\00\00\00\00\00\00" }>, align 8
@alloc_513570631223a12912d85da2bec3b15a = private unnamed_addr constant <{}> zeroinitializer, align 8
@alloc_b5970474149acb40bd55b9b54ff0a4d7 = private unnamed_addr constant <{ [75 x i8] }> <{ [75 x i8] c"/rustc/82e1608dfa6e0b5569232559e3d385fea5a93112/library/core/src/fmt/mod.rs" }>, align 1
@alloc_b3a1ae5691ce283efab06be3afc2c025 = private unnamed_addr constant <{ ptr, [16 x i8] }> <{ ptr @alloc_b5970474149acb40bd55b9b54ff0a4d7, [16 x i8] c"K\00\00\00\00\00\00\00M\01\00\00\0D\00\00\00" }>, align 8
@alloc_d988653bf5bec18a49312bc37b98a2c4 = private unnamed_addr constant <{ [80 x i8] }> <{ [80 x i8] c"/rustc/82e1608dfa6e0b5569232559e3d385fea5a93112/library/core/src/alloc/layout.rs" }>, align 1
@alloc_cd349b0f800e182091472053d83a063d = private unnamed_addr constant <{ ptr, [16 x i8] }> <{ ptr @alloc_d988653bf5bec18a49312bc37b98a2c4, [16 x i8] c"P\00\00\00\00\00\00\00\C1\01\00\00)\00\00\00" }>, align 8
@str.0 = internal constant [25 x i8] c"attempt to divide by zero"
@__rust_no_alloc_shim_is_unstable = external global i8
@alloc_ffa3cdb3ae88e54a1cc225f31dd07672 = private unnamed_addr constant <{ ptr, [8 x i8] }> <{ ptr @alloc_513570631223a12912d85da2bec3b15a, [8 x i8] zeroinitializer }>, align 8
@alloc_53973d2fe29b4adba8bb7390b5678745 = private unnamed_addr constant <{ [8 x i8] }> zeroinitializer, align 8

; core::fmt::Arguments::new_v1
; Function Attrs: inlinehint nonlazybind uwtable
define internal void @_ZN4core3fmt9Arguments6new_v117hcb77c7334b09c897E(ptr sret(%"core::fmt::Arguments<'_>") align 8 %_0, ptr align 8 %pieces.0, i64 %pieces.1, ptr align 8 %args.0, i64 %args.1) unnamed_addr #0 {
start:
  %_15 = alloca { ptr, i64 }, align 8
  %_13 = alloca { ptr, i64 }, align 8
  %_11 = alloca %"core::fmt::Arguments<'_>", align 8
  %_3 = icmp ult i64 %pieces.1, %args.1
  br i1 %_3, label %bb1, label %bb2

bb2:                                              ; preds = %start
  %_8 = add i64 %args.1, 1
  %_6 = icmp ugt i64 %pieces.1, %_8
  br i1 %_6, label %bb3, label %bb4

bb1:                                              ; preds = %start
  br label %bb3

bb4:                                              ; preds = %bb2
  store ptr null, ptr %_13, align 8
  %0 = getelementptr inbounds { ptr, i64 }, ptr %_0, i32 0, i32 0
  store ptr %pieces.0, ptr %0, align 8
  %1 = getelementptr inbounds { ptr, i64 }, ptr %_0, i32 0, i32 1
  store i64 %pieces.1, ptr %1, align 8
  %2 = getelementptr inbounds { ptr, i64 }, ptr %_13, i32 0, i32 0
  %3 = load ptr, ptr %2, align 8, !align !3, !noundef !4
  %4 = getelementptr inbounds { ptr, i64 }, ptr %_13, i32 0, i32 1
  %5 = load i64, ptr %4, align 8
  %6 = getelementptr inbounds %"core::fmt::Arguments<'_>", ptr %_0, i32 0, i32 2
  %7 = getelementptr inbounds { ptr, i64 }, ptr %6, i32 0, i32 0
  store ptr %3, ptr %7, align 8
  %8 = getelementptr inbounds { ptr, i64 }, ptr %6, i32 0, i32 1
  store i64 %5, ptr %8, align 8
  %9 = getelementptr inbounds %"core::fmt::Arguments<'_>", ptr %_0, i32 0, i32 1
  %10 = getelementptr inbounds { ptr, i64 }, ptr %9, i32 0, i32 0
  store ptr %args.0, ptr %10, align 8
  %11 = getelementptr inbounds { ptr, i64 }, ptr %9, i32 0, i32 1
  store i64 %args.1, ptr %11, align 8
  ret void

bb3:                                              ; preds = %bb1, %bb2
  store ptr null, ptr %_15, align 8
  %12 = getelementptr inbounds { ptr, i64 }, ptr %_11, i32 0, i32 0
  store ptr @alloc_560206a49c61adca6f3f0639a12632eb, ptr %12, align 8
  %13 = getelementptr inbounds { ptr, i64 }, ptr %_11, i32 0, i32 1
  store i64 1, ptr %13, align 8
  %14 = getelementptr inbounds { ptr, i64 }, ptr %_15, i32 0, i32 0
  %15 = load ptr, ptr %14, align 8, !align !3, !noundef !4
  %16 = getelementptr inbounds { ptr, i64 }, ptr %_15, i32 0, i32 1
  %17 = load i64, ptr %16, align 8
  %18 = getelementptr inbounds %"core::fmt::Arguments<'_>", ptr %_11, i32 0, i32 2
  %19 = getelementptr inbounds { ptr, i64 }, ptr %18, i32 0, i32 0
  store ptr %15, ptr %19, align 8
  %20 = getelementptr inbounds { ptr, i64 }, ptr %18, i32 0, i32 1
  store i64 %17, ptr %20, align 8
  %21 = getelementptr inbounds %"core::fmt::Arguments<'_>", ptr %_11, i32 0, i32 1
  %22 = getelementptr inbounds { ptr, i64 }, ptr %21, i32 0, i32 0
  store ptr @alloc_513570631223a12912d85da2bec3b15a, ptr %22, align 8
  %23 = getelementptr inbounds { ptr, i64 }, ptr %21, i32 0, i32 1
  store i64 0, ptr %23, align 8
; call core::panicking::panic_fmt
  call void @_ZN4core9panicking9panic_fmt17hbf0e066aabfa482cE(ptr align 8 %_11, ptr align 8 @alloc_b3a1ae5691ce283efab06be3afc2c025) #12
  unreachable
}

; core::ops::function::FnOnce::call_once
; Function Attrs: inlinehint nonlazybind uwtable
define internal void @_ZN4core3ops8function6FnOnce9call_once17he1527e6e8d2f7e44E(ptr sret(%"alloc::string::String") align 8 %_0, ptr align 1 %0, i64 %1) unnamed_addr #0 {
start:
  %_2 = alloca { ptr, i64 }, align 8
  %2 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 0
  store ptr %0, ptr %2, align 8
  %3 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 1
  store i64 %1, ptr %3, align 8
  %4 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 0
  %5 = load ptr, ptr %4, align 8, !nonnull !4, !align !5, !noundef !4
  %6 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 1
  %7 = load i64, ptr %6, align 8, !noundef !4
; call alloc::str::<impl alloc::borrow::ToOwned for str>::to_owned
  call void @"_ZN5alloc3str56_$LT$impl$u20$alloc..borrow..ToOwned$u20$for$u20$str$GT$8to_owned17hec9de8ccf5c60281E"(ptr sret(%"alloc::string::String") align 8 %_0, ptr align 1 %5, i64 %7)
  ret void
}

; core::ptr::drop_in_place<alloc::string::String>
; Function Attrs: nonlazybind uwtable
define void @"_ZN4core3ptr42drop_in_place$LT$alloc..string..String$GT$17h2a2332ac5eb62d91E"(ptr align 8 %_1) unnamed_addr #1 {
start:
; call core::ptr::drop_in_place<alloc::vec::Vec<u8>>
  call void @"_ZN4core3ptr46drop_in_place$LT$alloc..vec..Vec$LT$u8$GT$$GT$17h05274eca465778a7E"(ptr align 8 %_1)
  ret void
}

; core::ptr::drop_in_place<alloc::vec::Vec<u8>>
; Function Attrs: nonlazybind uwtable
define void @"_ZN4core3ptr46drop_in_place$LT$alloc..vec..Vec$LT$u8$GT$$GT$17h05274eca465778a7E"(ptr align 8 %_1) unnamed_addr #1 personality ptr @rust_eh_personality {
start:
  %0 = alloca { ptr, i32 }, align 8
; invoke <alloc::vec::Vec<T,A> as core::ops::drop::Drop>::drop
  invoke void @"_ZN70_$LT$alloc..vec..Vec$LT$T$C$A$GT$$u20$as$u20$core..ops..drop..Drop$GT$4drop17h9beba6c359e0e338E"(ptr align 8 %_1)
          to label %bb4 unwind label %cleanup

bb3:                                              ; preds = %cleanup
; invoke core::ptr::drop_in_place<alloc::raw_vec::RawVec<u8>>
  invoke void @"_ZN4core3ptr53drop_in_place$LT$alloc..raw_vec..RawVec$LT$u8$GT$$GT$17h0e905d5e11ff23a8E"(ptr align 8 %_1) #13
          to label %bb1 unwind label %terminate

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
  call void @"_ZN4core3ptr53drop_in_place$LT$alloc..raw_vec..RawVec$LT$u8$GT$$GT$17h0e905d5e11ff23a8E"(ptr align 8 %_1)
  ret void

terminate:                                        ; preds = %bb3
  %6 = landingpad { ptr, i32 }
          filter [0 x ptr] zeroinitializer
  %7 = extractvalue { ptr, i32 } %6, 0
  %8 = extractvalue { ptr, i32 } %6, 1
; call core::panicking::panic_in_cleanup
  call void @_ZN4core9panicking16panic_in_cleanup17hd44bb2114362504eE() #14
  unreachable

bb1:                                              ; preds = %bb3
  %9 = load ptr, ptr %0, align 8, !noundef !4
  %10 = getelementptr inbounds { ptr, i32 }, ptr %0, i32 0, i32 1
  %11 = load i32, ptr %10, align 8, !noundef !4
  %12 = insertvalue { ptr, i32 } poison, ptr %9, 0
  %13 = insertvalue { ptr, i32 } %12, i32 %11, 1
  resume { ptr, i32 } %13
}

; core::ptr::drop_in_place<alloc::raw_vec::RawVec<u8>>
; Function Attrs: nonlazybind uwtable
define void @"_ZN4core3ptr53drop_in_place$LT$alloc..raw_vec..RawVec$LT$u8$GT$$GT$17h0e905d5e11ff23a8E"(ptr align 8 %_1) unnamed_addr #1 {
start:
; call <alloc::raw_vec::RawVec<T,A> as core::ops::drop::Drop>::drop
  call void @"_ZN77_$LT$alloc..raw_vec..RawVec$LT$T$C$A$GT$$u20$as$u20$core..ops..drop..Drop$GT$4drop17hec8ad348e6c6cc12E"(ptr align 8 %_1)
  ret void
}

; core::alloc::layout::Layout::array::inner
; Function Attrs: inlinehint nonlazybind uwtable
define internal { i64, i64 } @_ZN4core5alloc6layout6Layout5array5inner17h81441bf7ab49fd96E(i64 %element_size, i64 %align, i64 %n) unnamed_addr #0 {
start:
  %_18 = alloca i64, align 8
  %_13 = alloca i64, align 8
  %_9 = alloca { i64, i64 }, align 8
  %_0 = alloca { i64, i64 }, align 8
  %0 = icmp eq i64 %element_size, 0
  br i1 %0, label %bb5, label %bb1

bb5:                                              ; preds = %bb4, %start
  %array_size = mul i64 %element_size, %n
  store i64 %align, ptr %_18, align 8
  %_19 = load i64, ptr %_18, align 8, !range !6, !noundef !4
  %_20 = icmp uge i64 %_19, 1
  %_21 = icmp ule i64 %_19, -9223372036854775808
  %_22 = and i1 %_20, %_21
  call void @llvm.assume(i1 %_22)
  %1 = getelementptr inbounds { i64, i64 }, ptr %_9, i32 0, i32 1
  store i64 %array_size, ptr %1, align 8
  store i64 %_19, ptr %_9, align 8
  %2 = getelementptr inbounds { i64, i64 }, ptr %_9, i32 0, i32 0
  %3 = load i64, ptr %2, align 8, !range !6, !noundef !4
  %4 = getelementptr inbounds { i64, i64 }, ptr %_9, i32 0, i32 1
  %5 = load i64, ptr %4, align 8, !noundef !4
  %6 = getelementptr inbounds { i64, i64 }, ptr %_0, i32 0, i32 0
  store i64 %3, ptr %6, align 8
  %7 = getelementptr inbounds { i64, i64 }, ptr %_0, i32 0, i32 1
  store i64 %5, ptr %7, align 8
  br label %bb6

bb1:                                              ; preds = %start
  store i64 %align, ptr %_13, align 8
  %_14 = load i64, ptr %_13, align 8, !range !6, !noundef !4
  %_15 = icmp uge i64 %_14, 1
  %_16 = icmp ule i64 %_14, -9223372036854775808
  %_17 = and i1 %_15, %_16
  call void @llvm.assume(i1 %_17)
  %_11 = sub i64 %_14, 1
  %_6 = sub i64 9223372036854775807, %_11
  %_7 = icmp eq i64 %element_size, 0
  %8 = call i1 @llvm.expect.i1(i1 %_7, i1 false)
  br i1 %8, label %panic, label %bb2

bb2:                                              ; preds = %bb1
  %_5 = udiv i64 %_6, %element_size
  %_4 = icmp ugt i64 %n, %_5
  br i1 %_4, label %bb3, label %bb4

panic:                                            ; preds = %bb1
; call core::panicking::panic
  call void @_ZN4core9panicking5panic17h195fc2a96298d4c3E(ptr align 1 @str.0, i64 25, ptr align 8 @alloc_cd349b0f800e182091472053d83a063d) #12
  unreachable

bb4:                                              ; preds = %bb2
  br label %bb5

bb3:                                              ; preds = %bb2
  store i64 0, ptr %_0, align 8
  br label %bb6

bb6:                                              ; preds = %bb3, %bb5
  %9 = getelementptr inbounds { i64, i64 }, ptr %_0, i32 0, i32 0
  %10 = load i64, ptr %9, align 8, !range !7, !noundef !4
  %11 = getelementptr inbounds { i64, i64 }, ptr %_0, i32 0, i32 1
  %12 = load i64, ptr %11, align 8
  %13 = insertvalue { i64, i64 } poison, i64 %10, 0
  %14 = insertvalue { i64, i64 } %13, i64 %12, 1
  ret { i64, i64 } %14
}

; core::option::Option<T>::map_or_else
; Function Attrs: inlinehint nonlazybind uwtable
define void @"_ZN4core6option15Option$LT$T$GT$11map_or_else17h4020f5759ef73b7aE"(ptr sret(%"alloc::string::String") align 8 %_0, ptr align 1 %0, i64 %1, ptr align 8 %default) unnamed_addr #0 personality ptr @rust_eh_personality {
start:
  %2 = alloca { ptr, i32 }, align 8
  %_10 = alloca i8, align 1
  %_9 = alloca i8, align 1
  %_7 = alloca { ptr, i64 }, align 8
  %self = alloca { ptr, i64 }, align 8
  %3 = getelementptr inbounds { ptr, i64 }, ptr %self, i32 0, i32 0
  store ptr %0, ptr %3, align 8
  %4 = getelementptr inbounds { ptr, i64 }, ptr %self, i32 0, i32 1
  store i64 %1, ptr %4, align 8
  store i8 1, ptr %_10, align 1
  store i8 1, ptr %_9, align 1
  %5 = load ptr, ptr %self, align 8, !noundef !4
  %6 = ptrtoint ptr %5 to i64
  %7 = icmp eq i64 %6, 0
  %_4 = select i1 %7, i64 0, i64 1
  %8 = icmp eq i64 %_4, 0
  br i1 %8, label %bb1, label %bb3

bb1:                                              ; preds = %start
  store i8 0, ptr %_10, align 1
; invoke alloc::fmt::format::{{closure}}
  invoke void @"_ZN5alloc3fmt6format28_$u7b$$u7b$closure$u7d$$u7d$17h485b1b4a811714c9E"(ptr sret(%"alloc::string::String") align 8 %_0, ptr align 8 %default)
          to label %bb5 unwind label %cleanup

bb3:                                              ; preds = %start
  %9 = getelementptr inbounds { ptr, i64 }, ptr %self, i32 0, i32 0
  %t.0 = load ptr, ptr %9, align 8, !nonnull !4, !align !5, !noundef !4
  %10 = getelementptr inbounds { ptr, i64 }, ptr %self, i32 0, i32 1
  %t.1 = load i64, ptr %10, align 8, !noundef !4
  store i8 0, ptr %_9, align 1
  %11 = getelementptr inbounds { ptr, i64 }, ptr %_7, i32 0, i32 0
  store ptr %t.0, ptr %11, align 8
  %12 = getelementptr inbounds { ptr, i64 }, ptr %_7, i32 0, i32 1
  store i64 %t.1, ptr %12, align 8
  %13 = getelementptr inbounds { ptr, i64 }, ptr %_7, i32 0, i32 0
  %14 = load ptr, ptr %13, align 8, !nonnull !4, !align !5, !noundef !4
  %15 = getelementptr inbounds { ptr, i64 }, ptr %_7, i32 0, i32 1
  %16 = load i64, ptr %15, align 8, !noundef !4
; invoke core::ops::function::FnOnce::call_once
  invoke void @_ZN4core3ops8function6FnOnce9call_once17he1527e6e8d2f7e44E(ptr sret(%"alloc::string::String") align 8 %_0, ptr align 1 %14, i64 %16)
          to label %bb4 unwind label %cleanup

bb14:                                             ; preds = %cleanup
  %17 = load i8, ptr %_9, align 1, !range !8, !noundef !4
  %18 = trunc i8 %17 to i1
  br i1 %18, label %bb13, label %bb8

cleanup:                                          ; preds = %bb3, %bb1
  %19 = landingpad { ptr, i32 }
          cleanup
  %20 = extractvalue { ptr, i32 } %19, 0
  %21 = extractvalue { ptr, i32 } %19, 1
  %22 = getelementptr inbounds { ptr, i32 }, ptr %2, i32 0, i32 0
  store ptr %20, ptr %22, align 8
  %23 = getelementptr inbounds { ptr, i32 }, ptr %2, i32 0, i32 1
  store i32 %21, ptr %23, align 8
  br label %bb14

bb5:                                              ; preds = %bb1
  br label %bb11

bb11:                                             ; preds = %bb4, %bb5
  %24 = load i8, ptr %_9, align 1, !range !8, !noundef !4
  %25 = trunc i8 %24 to i1
  br i1 %25, label %bb10, label %bb6

bb4:                                              ; preds = %bb3
  br label %bb11

bb6:                                              ; preds = %bb10, %bb11
  %26 = load i8, ptr %_10, align 1, !range !8, !noundef !4
  %27 = trunc i8 %26 to i1
  br i1 %27, label %bb12, label %bb7

bb10:                                             ; preds = %bb11
  br label %bb6

bb7:                                              ; preds = %bb12, %bb6
  ret void

bb12:                                             ; preds = %bb6
  br label %bb7

bb8:                                              ; preds = %bb13, %bb14
  %28 = load i8, ptr %_10, align 1, !range !8, !noundef !4
  %29 = trunc i8 %28 to i1
  br i1 %29, label %bb15, label %bb9

bb13:                                             ; preds = %bb14
  br label %bb8

bb9:                                              ; preds = %bb15, %bb8
  %30 = load ptr, ptr %2, align 8, !noundef !4
  %31 = getelementptr inbounds { ptr, i32 }, ptr %2, i32 0, i32 1
  %32 = load i32, ptr %31, align 8, !noundef !4
  %33 = insertvalue { ptr, i32 } poison, ptr %30, 0
  %34 = insertvalue { ptr, i32 } %33, i32 %32, 1
  resume { ptr, i32 } %34

bb15:                                             ; preds = %bb8
  br label %bb9

bb2:                                              ; No predecessors!
  unreachable
}

; <T as alloc::slice::hack::ConvertVec>::to_vec
; Function Attrs: inlinehint nonlazybind uwtable
define void @"_ZN52_$LT$T$u20$as$u20$alloc..slice..hack..ConvertVec$GT$6to_vec17hb1888feabb4d4ffcE"(ptr sret(%"alloc::vec::Vec<u8>") align 8 %_0, ptr align 1 %s.0, i64 %s.1) unnamed_addr #0 personality ptr @rust_eh_personality {
start:
  %0 = alloca { ptr, i32 }, align 8
  %v = alloca %"alloc::vec::Vec<u8>", align 8
; invoke alloc::raw_vec::RawVec<T,A>::allocate_in
  %1 = invoke { ptr, i64 } @"_ZN5alloc7raw_vec19RawVec$LT$T$C$A$GT$11allocate_in17he6cff2b02595206cE"(i64 %s.1, i1 zeroext false)
          to label %bb4 unwind label %cleanup

bb3:                                              ; preds = %cleanup
  br i1 false, label %bb2, label %bb1

cleanup:                                          ; preds = %start
  %2 = landingpad { ptr, i32 }
          cleanup
  %3 = extractvalue { ptr, i32 } %2, 0
  %4 = extractvalue { ptr, i32 } %2, 1
  %5 = getelementptr inbounds { ptr, i32 }, ptr %0, i32 0, i32 0
  store ptr %3, ptr %5, align 8
  %6 = getelementptr inbounds { ptr, i32 }, ptr %0, i32 0, i32 1
  store i32 %4, ptr %6, align 8
  br label %bb3

bb4:                                              ; preds = %start
  %_13.0 = extractvalue { ptr, i64 } %1, 0
  %_13.1 = extractvalue { ptr, i64 } %1, 1
  %7 = getelementptr inbounds { ptr, i64 }, ptr %v, i32 0, i32 0
  store ptr %_13.0, ptr %7, align 8
  %8 = getelementptr inbounds { ptr, i64 }, ptr %v, i32 0, i32 1
  store i64 %_13.1, ptr %8, align 8
  %9 = getelementptr inbounds %"alloc::vec::Vec<u8>", ptr %v, i32 0, i32 1
  store i64 0, ptr %9, align 8
  %self = load ptr, ptr %v, align 8, !nonnull !4, !noundef !4
  %10 = mul i64 %s.1, 1
  call void @llvm.memcpy.p0.p0.i64(ptr align 1 %self, ptr align 1 %s.0, i64 %10, i1 false)
  %11 = getelementptr inbounds %"alloc::vec::Vec<u8>", ptr %v, i32 0, i32 1
  store i64 %s.1, ptr %11, align 8
  call void @llvm.memcpy.p0.p0.i64(ptr align 8 %_0, ptr align 8 %v, i64 24, i1 false)
  ret void

bb1:                                              ; preds = %bb2, %bb3
  %12 = load ptr, ptr %0, align 8, !noundef !4
  %13 = getelementptr inbounds { ptr, i32 }, ptr %0, i32 0, i32 1
  %14 = load i32, ptr %13, align 8, !noundef !4
  %15 = insertvalue { ptr, i32 } poison, ptr %12, 0
  %16 = insertvalue { ptr, i32 } %15, i32 %14, 1
  resume { ptr, i32 } %16

bb2:                                              ; preds = %bb3
  br label %bb1
}

; alloc::fmt::format
; Function Attrs: inlinehint nonlazybind uwtable
define internal void @_ZN5alloc3fmt6format17hde97e0b1aee510c6E(ptr sret(%"alloc::string::String") align 8 %_0, ptr align 8 %args) unnamed_addr #0 {
start:
  %_4 = alloca ptr, align 8
  %_2 = alloca { ptr, i64 }, align 8
  %0 = getelementptr inbounds { ptr, i64 }, ptr %args, i32 0, i32 0
  %_6.0 = load ptr, ptr %0, align 8, !nonnull !4, !align !3, !noundef !4
  %1 = getelementptr inbounds { ptr, i64 }, ptr %args, i32 0, i32 1
  %_6.1 = load i64, ptr %1, align 8, !noundef !4
  %2 = getelementptr inbounds %"core::fmt::Arguments<'_>", ptr %args, i32 0, i32 1
  %3 = getelementptr inbounds { ptr, i64 }, ptr %2, i32 0, i32 0
  %_7.0 = load ptr, ptr %3, align 8, !nonnull !4, !align !3, !noundef !4
  %4 = getelementptr inbounds { ptr, i64 }, ptr %2, i32 0, i32 1
  %_7.1 = load i64, ptr %4, align 8, !noundef !4
  %5 = icmp eq i64 %_6.1, 0
  br i1 %5, label %bb3, label %bb5

bb3:                                              ; preds = %start
  %6 = icmp eq i64 %_7.1, 0
  br i1 %6, label %bb7, label %bb4

bb5:                                              ; preds = %start
  %7 = icmp eq i64 %_6.1, 1
  br i1 %7, label %bb6, label %bb4

bb7:                                              ; preds = %bb3
  %8 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 0
  store ptr @alloc_513570631223a12912d85da2bec3b15a, ptr %8, align 8
  %9 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 1
  store i64 0, ptr %9, align 8
  br label %bb1

bb4:                                              ; preds = %bb6, %bb5, %bb3
  store ptr null, ptr %_2, align 8
  br label %bb1

bb1:                                              ; preds = %bb4, %bb8, %bb7
  store ptr %args, ptr %_4, align 8
  %10 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 0
  %11 = load ptr, ptr %10, align 8, !align !5, !noundef !4
  %12 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 1
  %13 = load i64, ptr %12, align 8
  %14 = load ptr, ptr %_4, align 8, !nonnull !4, !align !3, !noundef !4
; call core::option::Option<T>::map_or_else
  call void @"_ZN4core6option15Option$LT$T$GT$11map_or_else17h4020f5759ef73b7aE"(ptr sret(%"alloc::string::String") align 8 %_0, ptr align 1 %11, i64 %13, ptr align 8 %14)
  ret void

bb6:                                              ; preds = %bb5
  %15 = icmp eq i64 %_7.1, 0
  br i1 %15, label %bb8, label %bb4

bb8:                                              ; preds = %bb6
  %s = getelementptr inbounds [0 x { ptr, i64 }], ptr %_6.0, i64 0, i64 0
  %16 = getelementptr inbounds [0 x { ptr, i64 }], ptr %_6.0, i64 0, i64 0
  %17 = getelementptr inbounds { ptr, i64 }, ptr %16, i32 0, i32 0
  %_14.0 = load ptr, ptr %17, align 8, !nonnull !4, !align !5, !noundef !4
  %18 = getelementptr inbounds { ptr, i64 }, ptr %16, i32 0, i32 1
  %_14.1 = load i64, ptr %18, align 8, !noundef !4
  %19 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 0
  store ptr %_14.0, ptr %19, align 8
  %20 = getelementptr inbounds { ptr, i64 }, ptr %_2, i32 0, i32 1
  store i64 %_14.1, ptr %20, align 8
  br label %bb1
}

; alloc::fmt::format::{{closure}}
; Function Attrs: inlinehint nonlazybind uwtable
define void @"_ZN5alloc3fmt6format28_$u7b$$u7b$closure$u7d$$u7d$17h485b1b4a811714c9E"(ptr sret(%"alloc::string::String") align 8 %_0, ptr align 8 %0) unnamed_addr #0 {
start:
  %_2 = alloca %"core::fmt::Arguments<'_>", align 8
  %_1 = alloca ptr, align 8
  store ptr %0, ptr %_1, align 8
  %_3 = load ptr, ptr %_1, align 8, !nonnull !4, !align !3, !noundef !4
  call void @llvm.memcpy.p0.p0.i64(ptr align 8 %_2, ptr align 8 %_3, i64 48, i1 false)
; call alloc::fmt::format::format_inner
  call void @_ZN5alloc3fmt6format12format_inner17hf6fc27e89bdb9586E(ptr sret(%"alloc::string::String") align 8 %_0, ptr align 8 %_2)
  ret void
}

; alloc::str::<impl alloc::borrow::ToOwned for str>::to_owned
; Function Attrs: inlinehint nonlazybind uwtable
define internal void @"_ZN5alloc3str56_$LT$impl$u20$alloc..borrow..ToOwned$u20$for$u20$str$GT$8to_owned17hec9de8ccf5c60281E"(ptr sret(%"alloc::string::String") align 8 %_0, ptr align 1 %self.0, i64 %self.1) unnamed_addr #0 {
start:
  %bytes = alloca %"alloc::vec::Vec<u8>", align 8
; call <T as alloc::slice::hack::ConvertVec>::to_vec
  call void @"_ZN52_$LT$T$u20$as$u20$alloc..slice..hack..ConvertVec$GT$6to_vec17hb1888feabb4d4ffcE"(ptr sret(%"alloc::vec::Vec<u8>") align 8 %bytes, ptr align 1 %self.0, i64 %self.1)
  call void @llvm.memcpy.p0.p0.i64(ptr align 8 %_0, ptr align 8 %bytes, i64 24, i1 false)
  ret void
}

; alloc::alloc::Global::alloc_impl
; Function Attrs: inlinehint nonlazybind uwtable
define internal { ptr, i64 } @_ZN5alloc5alloc6Global10alloc_impl17h3ce533e4ae0a1c26E(ptr align 1 %self, i64 %0, i64 %1, i1 zeroext %zeroed) unnamed_addr #0 {
start:
  %2 = alloca i8, align 1
  %_76 = alloca { ptr, i64 }, align 8
  %_75 = alloca %"core::ptr::metadata::PtrRepr<[u8]>", align 8
  %_62 = alloca ptr, align 8
  %_57 = alloca i64, align 8
  %_43 = alloca i64, align 8
  %_34 = alloca { ptr, i64 }, align 8
  %_33 = alloca %"core::ptr::metadata::PtrRepr<[u8]>", align 8
  %_22 = alloca i64, align 8
  %_18 = alloca { ptr, i64 }, align 8
  %self4 = alloca ptr, align 8
  %self3 = alloca ptr, align 8
  %_12 = alloca ptr, align 8
  %layout2 = alloca { i64, i64 }, align 8
  %layout1 = alloca { i64, i64 }, align 8
  %raw_ptr = alloca ptr, align 8
  %data = alloca ptr, align 8
  %_6 = alloca { ptr, i64 }, align 8
  %_0 = alloca { ptr, i64 }, align 8
  %layout = alloca { i64, i64 }, align 8
  %3 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  store i64 %0, ptr %3, align 8
  %4 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  store i64 %1, ptr %4, align 8
  %5 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %size = load i64, ptr %5, align 8, !noundef !4
  %6 = icmp eq i64 %size, 0
  br i1 %6, label %bb2, label %bb1

bb2:                                              ; preds = %start
  %self5 = load i64, ptr %layout, align 8, !range !6, !noundef !4
  store i64 %self5, ptr %_22, align 8
  %_23 = load i64, ptr %_22, align 8, !range !6, !noundef !4
  %_24 = icmp uge i64 %_23, 1
  %_25 = icmp ule i64 %_23, -9223372036854775808
  %_26 = and i1 %_24, %_25
  call void @llvm.assume(i1 %_26)
  %ptr = inttoptr i64 %_23 to ptr
  store ptr %ptr, ptr %data, align 8
  %_31 = load ptr, ptr %data, align 8, !noundef !4
  store ptr %_31, ptr %_34, align 8
  %7 = getelementptr inbounds { ptr, i64 }, ptr %_34, i32 0, i32 1
  store i64 0, ptr %7, align 8
  %8 = getelementptr inbounds { ptr, i64 }, ptr %_34, i32 0, i32 0
  %9 = load ptr, ptr %8, align 8, !noundef !4
  %10 = getelementptr inbounds { ptr, i64 }, ptr %_34, i32 0, i32 1
  %11 = load i64, ptr %10, align 8, !noundef !4
  %12 = getelementptr inbounds { ptr, i64 }, ptr %_33, i32 0, i32 0
  store ptr %9, ptr %12, align 8
  %13 = getelementptr inbounds { ptr, i64 }, ptr %_33, i32 0, i32 1
  store i64 %11, ptr %13, align 8
  %14 = getelementptr inbounds { ptr, i64 }, ptr %_33, i32 0, i32 0
  %ptr.0 = load ptr, ptr %14, align 8, !noundef !4
  %15 = getelementptr inbounds { ptr, i64 }, ptr %_33, i32 0, i32 1
  %ptr.1 = load i64, ptr %15, align 8, !noundef !4
  %16 = getelementptr inbounds { ptr, i64 }, ptr %_6, i32 0, i32 0
  store ptr %ptr.0, ptr %16, align 8
  %17 = getelementptr inbounds { ptr, i64 }, ptr %_6, i32 0, i32 1
  store i64 %ptr.1, ptr %17, align 8
  %18 = getelementptr inbounds { ptr, i64 }, ptr %_6, i32 0, i32 0
  %19 = load ptr, ptr %18, align 8, !nonnull !4, !noundef !4
  %20 = getelementptr inbounds { ptr, i64 }, ptr %_6, i32 0, i32 1
  %21 = load i64, ptr %20, align 8, !noundef !4
  %22 = getelementptr inbounds { ptr, i64 }, ptr %_0, i32 0, i32 0
  store ptr %19, ptr %22, align 8
  %23 = getelementptr inbounds { ptr, i64 }, ptr %_0, i32 0, i32 1
  store i64 %21, ptr %23, align 8
  br label %bb11

bb1:                                              ; preds = %start
  br i1 %zeroed, label %bb3, label %bb4

bb11:                                             ; preds = %bb10, %bb8, %bb2
  %24 = getelementptr inbounds { ptr, i64 }, ptr %_0, i32 0, i32 0
  %25 = load ptr, ptr %24, align 8, !noundef !4
  %26 = getelementptr inbounds { ptr, i64 }, ptr %_0, i32 0, i32 1
  %27 = load i64, ptr %26, align 8
  %28 = insertvalue { ptr, i64 } poison, ptr %25, 0
  %29 = insertvalue { ptr, i64 } %28, i64 %27, 1
  ret { ptr, i64 } %29

bb4:                                              ; preds = %bb1
  %30 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  %31 = load i64, ptr %30, align 8, !range !6, !noundef !4
  %32 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %33 = load i64, ptr %32, align 8, !noundef !4
  %34 = getelementptr inbounds { i64, i64 }, ptr %layout2, i32 0, i32 0
  store i64 %31, ptr %34, align 8
  %35 = getelementptr inbounds { i64, i64 }, ptr %layout2, i32 0, i32 1
  store i64 %33, ptr %35, align 8
  %36 = load volatile i8, ptr @__rust_no_alloc_shim_is_unstable, align 1
  store i8 %36, ptr %2, align 1
  %_48 = load i8, ptr %2, align 1, !noundef !4
  %37 = getelementptr inbounds { i64, i64 }, ptr %layout2, i32 0, i32 1
  %_51 = load i64, ptr %37, align 8, !noundef !4
  %self6 = load i64, ptr %layout2, align 8, !range !6, !noundef !4
  store i64 %self6, ptr %_57, align 8
  %_58 = load i64, ptr %_57, align 8, !range !6, !noundef !4
  %_59 = icmp uge i64 %_58, 1
  %_60 = icmp ule i64 %_58, -9223372036854775808
  %_61 = and i1 %_59, %_60
  call void @llvm.assume(i1 %_61)
  %38 = call ptr @__rust_alloc(i64 %_51, i64 %_58) #15
  store ptr %38, ptr %raw_ptr, align 8
  br label %bb5

bb3:                                              ; preds = %bb1
  %39 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  %40 = load i64, ptr %39, align 8, !range !6, !noundef !4
  %41 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %42 = load i64, ptr %41, align 8, !noundef !4
  %43 = getelementptr inbounds { i64, i64 }, ptr %layout1, i32 0, i32 0
  store i64 %40, ptr %43, align 8
  %44 = getelementptr inbounds { i64, i64 }, ptr %layout1, i32 0, i32 1
  store i64 %42, ptr %44, align 8
  %45 = getelementptr inbounds { i64, i64 }, ptr %layout1, i32 0, i32 1
  %_38 = load i64, ptr %45, align 8, !noundef !4
  %self7 = load i64, ptr %layout1, align 8, !range !6, !noundef !4
  store i64 %self7, ptr %_43, align 8
  %_44 = load i64, ptr %_43, align 8, !range !6, !noundef !4
  %_45 = icmp uge i64 %_44, 1
  %_46 = icmp ule i64 %_44, -9223372036854775808
  %_47 = and i1 %_45, %_46
  call void @llvm.assume(i1 %_47)
  %46 = call ptr @__rust_alloc_zeroed(i64 %_38, i64 %_44) #15
  store ptr %46, ptr %raw_ptr, align 8
  br label %bb5

bb5:                                              ; preds = %bb3, %bb4
  %ptr8 = load ptr, ptr %raw_ptr, align 8, !noundef !4
  %_63 = ptrtoint ptr %ptr8 to i64
  %47 = icmp eq i64 %_63, 0
  br i1 %47, label %bb15, label %bb16

bb15:                                             ; preds = %bb5
  store ptr null, ptr %self4, align 8
  br label %bb6

bb16:                                             ; preds = %bb5
  store ptr %ptr8, ptr %_62, align 8
  %48 = load ptr, ptr %_62, align 8, !nonnull !4, !noundef !4
  store ptr %48, ptr %self4, align 8
  br label %bb6

bb6:                                              ; preds = %bb16, %bb15
  %49 = load ptr, ptr %self4, align 8, !noundef !4
  %50 = ptrtoint ptr %49 to i64
  %51 = icmp eq i64 %50, 0
  %_67 = select i1 %51, i64 0, i64 1
  %52 = icmp eq i64 %_67, 0
  br i1 %52, label %bb17, label %bb18

bb17:                                             ; preds = %bb6
  store ptr null, ptr %self3, align 8
  br label %bb19

bb18:                                             ; preds = %bb6
  %v = load ptr, ptr %self4, align 8, !nonnull !4, !noundef !4
  store ptr %v, ptr %self3, align 8
  br label %bb19

bb19:                                             ; preds = %bb18, %bb17
  %53 = load ptr, ptr %self3, align 8, !noundef !4
  %54 = ptrtoint ptr %53 to i64
  %55 = icmp eq i64 %54, 0
  %_69 = select i1 %55, i64 1, i64 0
  %56 = icmp eq i64 %_69, 0
  br i1 %56, label %bb21, label %bb20

bb21:                                             ; preds = %bb19
  %v9 = load ptr, ptr %self3, align 8, !nonnull !4, !noundef !4
  store ptr %v9, ptr %_12, align 8
  br label %bb7

bb20:                                             ; preds = %bb19
  store ptr null, ptr %_12, align 8
  br label %bb7

bb7:                                              ; preds = %bb20, %bb21
  %57 = load ptr, ptr %_12, align 8, !noundef !4
  %58 = ptrtoint ptr %57 to i64
  %59 = icmp eq i64 %58, 0
  %_16 = select i1 %59, i64 1, i64 0
  %60 = icmp eq i64 %_16, 0
  br i1 %60, label %bb8, label %bb10

bb8:                                              ; preds = %bb7
  %ptr10 = load ptr, ptr %_12, align 8, !nonnull !4, !noundef !4
  store ptr %ptr10, ptr %_76, align 8
  %61 = getelementptr inbounds { ptr, i64 }, ptr %_76, i32 0, i32 1
  store i64 %size, ptr %61, align 8
  %62 = getelementptr inbounds { ptr, i64 }, ptr %_76, i32 0, i32 0
  %63 = load ptr, ptr %62, align 8, !noundef !4
  %64 = getelementptr inbounds { ptr, i64 }, ptr %_76, i32 0, i32 1
  %65 = load i64, ptr %64, align 8, !noundef !4
  %66 = getelementptr inbounds { ptr, i64 }, ptr %_75, i32 0, i32 0
  store ptr %63, ptr %66, align 8
  %67 = getelementptr inbounds { ptr, i64 }, ptr %_75, i32 0, i32 1
  store i64 %65, ptr %67, align 8
  %68 = getelementptr inbounds { ptr, i64 }, ptr %_75, i32 0, i32 0
  %ptr.011 = load ptr, ptr %68, align 8, !noundef !4
  %69 = getelementptr inbounds { ptr, i64 }, ptr %_75, i32 0, i32 1
  %ptr.112 = load i64, ptr %69, align 8, !noundef !4
  %70 = getelementptr inbounds { ptr, i64 }, ptr %_18, i32 0, i32 0
  store ptr %ptr.011, ptr %70, align 8
  %71 = getelementptr inbounds { ptr, i64 }, ptr %_18, i32 0, i32 1
  store i64 %ptr.112, ptr %71, align 8
  %72 = getelementptr inbounds { ptr, i64 }, ptr %_18, i32 0, i32 0
  %73 = load ptr, ptr %72, align 8, !nonnull !4, !noundef !4
  %74 = getelementptr inbounds { ptr, i64 }, ptr %_18, i32 0, i32 1
  %75 = load i64, ptr %74, align 8, !noundef !4
  %76 = getelementptr inbounds { ptr, i64 }, ptr %_0, i32 0, i32 0
  store ptr %73, ptr %76, align 8
  %77 = getelementptr inbounds { ptr, i64 }, ptr %_0, i32 0, i32 1
  store i64 %75, ptr %77, align 8
  br label %bb11

bb10:                                             ; preds = %bb7
  store ptr null, ptr %_0, align 8
  br label %bb11

bb9:                                              ; No predecessors!
  unreachable
}

; alloc::raw_vec::RawVec<T,A>::allocate_in
; Function Attrs: nonlazybind uwtable
define { ptr, i64 } @"_ZN5alloc7raw_vec19RawVec$LT$T$C$A$GT$11allocate_in17he6cff2b02595206cE"(i64 %capacity, i1 zeroext %0) unnamed_addr #1 personality ptr @rust_eh_personality {
start:
  %1 = alloca { ptr, i32 }, align 8
  %_44 = alloca ptr, align 8
  %_29 = alloca ptr, align 8
  %_28 = alloca ptr, align 8
  %self = alloca ptr, align 8
  %_23 = alloca ptr, align 8
  %result = alloca { ptr, i64 }, align 8
  %_7 = alloca { i64, i64 }, align 8
  %layout = alloca { i64, i64 }, align 8
  %_0 = alloca { ptr, i64 }, align 8
  %alloc = alloca %"alloc::alloc::Global", align 1
  %init = alloca i8, align 1
  %2 = zext i1 %0 to i8
  store i8 %2, ptr %init, align 1
  br i1 false, label %bb2, label %bb1

bb1:                                              ; preds = %start
  %3 = icmp eq i64 %capacity, 0
  br i1 %3, label %bb2, label %bb3

bb2:                                              ; preds = %bb1, %start
  store ptr inttoptr (i64 1 to ptr), ptr %_29, align 8
  %4 = load ptr, ptr %_29, align 8, !nonnull !4, !noundef !4
  store ptr %4, ptr %_28, align 8
  %5 = load ptr, ptr %_28, align 8, !nonnull !4, !noundef !4
  store ptr %5, ptr %_0, align 8
  %6 = getelementptr inbounds { ptr, i64 }, ptr %_0, i32 0, i32 1
  store i64 0, ptr %6, align 8
  br label %bb14

bb3:                                              ; preds = %bb1
; invoke core::alloc::layout::Layout::array::inner
  %7 = invoke { i64, i64 } @_ZN4core5alloc6layout6Layout5array5inner17h81441bf7ab49fd96E(i64 1, i64 1, i64 %capacity)
          to label %bb18 unwind label %cleanup

bb17:                                             ; preds = %cleanup
  br i1 true, label %bb16, label %bb15

cleanup:                                          ; preds = %bb4, %bb12, %bb7, %bb8, %bb3
  %8 = landingpad { ptr, i32 }
          cleanup
  %9 = extractvalue { ptr, i32 } %8, 0
  %10 = extractvalue { ptr, i32 } %8, 1
  %11 = getelementptr inbounds { ptr, i32 }, ptr %1, i32 0, i32 0
  store ptr %9, ptr %11, align 8
  %12 = getelementptr inbounds { ptr, i32 }, ptr %1, i32 0, i32 1
  store i32 %10, ptr %12, align 8
  br label %bb17

bb18:                                             ; preds = %bb3
  store { i64, i64 } %7, ptr %_7, align 8
  %13 = load i64, ptr %_7, align 8, !range !7, !noundef !4
  %14 = icmp eq i64 %13, 0
  %_8 = select i1 %14, i64 1, i64 0
  %15 = icmp eq i64 %_8, 0
  br i1 %15, label %bb6, label %bb4

bb6:                                              ; preds = %bb18
  %16 = getelementptr inbounds { i64, i64 }, ptr %_7, i32 0, i32 0
  %layout.0 = load i64, ptr %16, align 8, !range !6, !noundef !4
  %17 = getelementptr inbounds { i64, i64 }, ptr %_7, i32 0, i32 1
  %layout.1 = load i64, ptr %17, align 8, !noundef !4
  %18 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  store i64 %layout.0, ptr %18, align 8
  %19 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  store i64 %layout.1, ptr %19, align 8
  %20 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %alloc_size = load i64, ptr %20, align 8, !noundef !4
  %21 = load i8, ptr %init, align 1, !range !8, !noundef !4
  %22 = trunc i8 %21 to i1
  %_14 = zext i1 %22 to i64
  %23 = icmp eq i64 %_14, 0
  br i1 %23, label %bb8, label %bb7

bb4:                                              ; preds = %bb18
; invoke alloc::raw_vec::capacity_overflow
  invoke void @_ZN5alloc7raw_vec17capacity_overflow17ha5d418fbc836d11cE() #12
          to label %unreachable unwind label %cleanup

bb8:                                              ; preds = %bb6
  %24 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  %_16.0 = load i64, ptr %24, align 8, !range !6, !noundef !4
  %25 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %_16.1 = load i64, ptr %25, align 8, !noundef !4
; invoke <alloc::alloc::Global as core::alloc::Allocator>::allocate
  %26 = invoke { ptr, i64 } @"_ZN63_$LT$alloc..alloc..Global$u20$as$u20$core..alloc..Allocator$GT$8allocate17hcfbfc4be4f9531bfE"(ptr align 1 %alloc, i64 %_16.0, i64 %_16.1)
          to label %bb9 unwind label %cleanup

bb7:                                              ; preds = %bb6
  %27 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  %_18.0 = load i64, ptr %27, align 8, !range !6, !noundef !4
  %28 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %_18.1 = load i64, ptr %28, align 8, !noundef !4
; invoke <alloc::alloc::Global as core::alloc::Allocator>::allocate_zeroed
  %29 = invoke { ptr, i64 } @"_ZN63_$LT$alloc..alloc..Global$u20$as$u20$core..alloc..Allocator$GT$15allocate_zeroed17h7453b81f8f970fb9E"(ptr align 1 %alloc, i64 %_18.0, i64 %_18.1)
          to label %bb10 unwind label %cleanup

bb9:                                              ; preds = %bb8
  store { ptr, i64 } %26, ptr %result, align 8
  br label %bb11

bb11:                                             ; preds = %bb10, %bb9
  %30 = load ptr, ptr %result, align 8, !noundef !4
  %31 = ptrtoint ptr %30 to i64
  %32 = icmp eq i64 %31, 0
  %_19 = select i1 %32, i64 1, i64 0
  %33 = icmp eq i64 %_19, 0
  br i1 %33, label %bb13, label %bb12

bb10:                                             ; preds = %bb7
  store { ptr, i64 } %29, ptr %result, align 8
  br label %bb11

bb13:                                             ; preds = %bb11
  %34 = getelementptr inbounds { ptr, i64 }, ptr %result, i32 0, i32 0
  %ptr.0 = load ptr, ptr %34, align 8, !nonnull !4, !noundef !4
  %35 = getelementptr inbounds { ptr, i64 }, ptr %result, i32 0, i32 1
  %ptr.1 = load i64, ptr %35, align 8, !noundef !4
  store ptr %ptr.0, ptr %self, align 8
  %_43 = load ptr, ptr %self, align 8, !noundef !4
  store ptr %_43, ptr %_44, align 8
  %36 = load ptr, ptr %_44, align 8, !nonnull !4, !noundef !4
  store ptr %36, ptr %_23, align 8
  %37 = load ptr, ptr %_23, align 8, !nonnull !4, !noundef !4
  store ptr %37, ptr %_0, align 8
  %38 = getelementptr inbounds { ptr, i64 }, ptr %_0, i32 0, i32 1
  store i64 %capacity, ptr %38, align 8
  br label %bb14

bb12:                                             ; preds = %bb11
  %39 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  %_22.0 = load i64, ptr %39, align 8, !range !6, !noundef !4
  %40 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %_22.1 = load i64, ptr %40, align 8, !noundef !4
; invoke alloc::alloc::handle_alloc_error
  invoke void @_ZN5alloc5alloc18handle_alloc_error17h7d585e222775d2b3E(i64 %_22.0, i64 %_22.1) #12
          to label %unreachable unwind label %cleanup

bb14:                                             ; preds = %bb2, %bb13
  %41 = getelementptr inbounds { ptr, i64 }, ptr %_0, i32 0, i32 0
  %42 = load ptr, ptr %41, align 8, !nonnull !4, !noundef !4
  %43 = getelementptr inbounds { ptr, i64 }, ptr %_0, i32 0, i32 1
  %44 = load i64, ptr %43, align 8, !noundef !4
  %45 = insertvalue { ptr, i64 } poison, ptr %42, 0
  %46 = insertvalue { ptr, i64 } %45, i64 %44, 1
  ret { ptr, i64 } %46

unreachable:                                      ; preds = %bb4, %bb12
  unreachable

bb5:                                              ; No predecessors!
  unreachable

bb15:                                             ; preds = %bb16, %bb17
  %47 = load ptr, ptr %1, align 8, !noundef !4
  %48 = getelementptr inbounds { ptr, i32 }, ptr %1, i32 0, i32 1
  %49 = load i32, ptr %48, align 8, !noundef !4
  %50 = insertvalue { ptr, i32 } poison, ptr %47, 0
  %51 = insertvalue { ptr, i32 } %50, i32 %49, 1
  resume { ptr, i32 } %51

bb16:                                             ; preds = %bb17
  br label %bb15
}

; alloc::raw_vec::RawVec<T,A>::current_memory
; Function Attrs: nonlazybind uwtable
define void @"_ZN5alloc7raw_vec19RawVec$LT$T$C$A$GT$14current_memory17hed0268053cefe6f7E"(ptr sret(%"core::option::Option<(core::ptr::non_null::NonNull<u8>, core::alloc::layout::Layout)>") align 8 %_0, ptr align 8 %self) unnamed_addr #1 {
start:
  %self2 = alloca ptr, align 8
  %self1 = alloca ptr, align 8
  %_10 = alloca ptr, align 8
  %_9 = alloca { ptr, { i64, i64 } }, align 8
  %layout = alloca { i64, i64 }, align 8
  br i1 false, label %bb2, label %bb1

bb1:                                              ; preds = %start
  %0 = getelementptr inbounds { ptr, i64 }, ptr %self, i32 0, i32 1
  %_3 = load i64, ptr %0, align 8, !noundef !4
  %1 = icmp eq i64 %_3, 0
  br i1 %1, label %bb2, label %bb3

bb2:                                              ; preds = %bb1, %start
  %2 = getelementptr inbounds %"core::option::Option<(core::ptr::non_null::NonNull<u8>, core::alloc::layout::Layout)>", ptr %_0, i32 0, i32 1
  store i64 0, ptr %2, align 8
  br label %bb4

bb3:                                              ; preds = %bb1
  %3 = getelementptr inbounds { ptr, i64 }, ptr %self, i32 0, i32 1
  %rhs = load i64, ptr %3, align 8, !noundef !4
  %size = mul nuw i64 1, %rhs
  %4 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  store i64 %size, ptr %4, align 8
  store i64 1, ptr %layout, align 8
  %self3 = load ptr, ptr %self, align 8, !nonnull !4, !noundef !4
  store ptr %self3, ptr %self1, align 8
  %_19 = load ptr, ptr %self1, align 8, !noundef !4
  store ptr %_19, ptr %self2, align 8
  %_24 = load ptr, ptr %self2, align 8, !noundef !4
  store ptr %_24, ptr %_10, align 8
  %5 = load ptr, ptr %_10, align 8, !nonnull !4, !noundef !4
  store ptr %5, ptr %_9, align 8
  %6 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  %7 = load i64, ptr %6, align 8, !range !6, !noundef !4
  %8 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %9 = load i64, ptr %8, align 8, !noundef !4
  %10 = getelementptr inbounds { ptr, { i64, i64 } }, ptr %_9, i32 0, i32 1
  %11 = getelementptr inbounds { i64, i64 }, ptr %10, i32 0, i32 0
  store i64 %7, ptr %11, align 8
  %12 = getelementptr inbounds { i64, i64 }, ptr %10, i32 0, i32 1
  store i64 %9, ptr %12, align 8
  call void @llvm.memcpy.p0.p0.i64(ptr align 8 %_0, ptr align 8 %_9, i64 24, i1 false)
  br label %bb4

bb4:                                              ; preds = %bb2, %bb3
  ret void
}

; <alloc::alloc::Global as core::alloc::Allocator>::deallocate
; Function Attrs: inlinehint nonlazybind uwtable
define internal void @"_ZN63_$LT$alloc..alloc..Global$u20$as$u20$core..alloc..Allocator$GT$10deallocate17h73cb97a91aa474baE"(ptr align 1 %self, ptr %ptr, i64 %0, i64 %1) unnamed_addr #0 {
start:
  %_14 = alloca i64, align 8
  %layout1 = alloca { i64, i64 }, align 8
  %layout = alloca { i64, i64 }, align 8
  %2 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  store i64 %0, ptr %2, align 8
  %3 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  store i64 %1, ptr %3, align 8
  %4 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %_4 = load i64, ptr %4, align 8, !noundef !4
  %5 = icmp eq i64 %_4, 0
  br i1 %5, label %bb2, label %bb1

bb2:                                              ; preds = %start
  br label %bb3

bb1:                                              ; preds = %start
  %6 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 0
  %7 = load i64, ptr %6, align 8, !range !6, !noundef !4
  %8 = getelementptr inbounds { i64, i64 }, ptr %layout, i32 0, i32 1
  %9 = load i64, ptr %8, align 8, !noundef !4
  %10 = getelementptr inbounds { i64, i64 }, ptr %layout1, i32 0, i32 0
  store i64 %7, ptr %10, align 8
  %11 = getelementptr inbounds { i64, i64 }, ptr %layout1, i32 0, i32 1
  store i64 %9, ptr %11, align 8
  %12 = getelementptr inbounds { i64, i64 }, ptr %layout1, i32 0, i32 1
  %_9 = load i64, ptr %12, align 8, !noundef !4
  %self2 = load i64, ptr %layout1, align 8, !range !6, !noundef !4
  store i64 %self2, ptr %_14, align 8
  %_15 = load i64, ptr %_14, align 8, !range !6, !noundef !4
  %_16 = icmp uge i64 %_15, 1
  %_17 = icmp ule i64 %_15, -9223372036854775808
  %_18 = and i1 %_16, %_17
  call void @llvm.assume(i1 %_18)
  call void @__rust_dealloc(ptr %ptr, i64 %_9, i64 %_15) #15
  br label %bb3

bb3:                                              ; preds = %bb1, %bb2
  ret void
}

; <alloc::alloc::Global as core::alloc::Allocator>::allocate_zeroed
; Function Attrs: inlinehint nonlazybind uwtable
define internal { ptr, i64 } @"_ZN63_$LT$alloc..alloc..Global$u20$as$u20$core..alloc..Allocator$GT$15allocate_zeroed17h7453b81f8f970fb9E"(ptr align 1 %self, i64 %layout.0, i64 %layout.1) unnamed_addr #0 {
start:
; call alloc::alloc::Global::alloc_impl
  %0 = call { ptr, i64 } @_ZN5alloc5alloc6Global10alloc_impl17h3ce533e4ae0a1c26E(ptr align 1 %self, i64 %layout.0, i64 %layout.1, i1 zeroext true)
  %_0.0 = extractvalue { ptr, i64 } %0, 0
  %_0.1 = extractvalue { ptr, i64 } %0, 1
  %1 = insertvalue { ptr, i64 } poison, ptr %_0.0, 0
  %2 = insertvalue { ptr, i64 } %1, i64 %_0.1, 1
  ret { ptr, i64 } %2
}

; <alloc::alloc::Global as core::alloc::Allocator>::allocate
; Function Attrs: inlinehint nonlazybind uwtable
define internal { ptr, i64 } @"_ZN63_$LT$alloc..alloc..Global$u20$as$u20$core..alloc..Allocator$GT$8allocate17hcfbfc4be4f9531bfE"(ptr align 1 %self, i64 %layout.0, i64 %layout.1) unnamed_addr #0 {
start:
; call alloc::alloc::Global::alloc_impl
  %0 = call { ptr, i64 } @_ZN5alloc5alloc6Global10alloc_impl17h3ce533e4ae0a1c26E(ptr align 1 %self, i64 %layout.0, i64 %layout.1, i1 zeroext false)
  %_0.0 = extractvalue { ptr, i64 } %0, 0
  %_0.1 = extractvalue { ptr, i64 } %0, 1
  %1 = insertvalue { ptr, i64 } poison, ptr %_0.0, 0
  %2 = insertvalue { ptr, i64 } %1, i64 %_0.1, 1
  ret { ptr, i64 } %2
}

; <alloc::vec::Vec<T,A> as core::ops::drop::Drop>::drop
; Function Attrs: nonlazybind uwtable
define void @"_ZN70_$LT$alloc..vec..Vec$LT$T$C$A$GT$$u20$as$u20$core..ops..drop..Drop$GT$4drop17h9beba6c359e0e338E"(ptr align 8 %self) unnamed_addr #1 {
start:
  %_10 = alloca { ptr, i64 }, align 8
  %_9 = alloca %"core::ptr::metadata::PtrRepr<[u8]>", align 8
  %self1 = load ptr, ptr %self, align 8, !nonnull !4, !noundef !4
  %0 = getelementptr inbounds %"alloc::vec::Vec<u8>", ptr %self, i32 0, i32 1
  %len = load i64, ptr %0, align 8, !noundef !4
  store ptr %self1, ptr %_10, align 8
  %1 = getelementptr inbounds { ptr, i64 }, ptr %_10, i32 0, i32 1
  store i64 %len, ptr %1, align 8
  %2 = getelementptr inbounds { ptr, i64 }, ptr %_10, i32 0, i32 0
  %3 = load ptr, ptr %2, align 8, !noundef !4
  %4 = getelementptr inbounds { ptr, i64 }, ptr %_10, i32 0, i32 1
  %5 = load i64, ptr %4, align 8, !noundef !4
  %6 = getelementptr inbounds { ptr, i64 }, ptr %_9, i32 0, i32 0
  store ptr %3, ptr %6, align 8
  %7 = getelementptr inbounds { ptr, i64 }, ptr %_9, i32 0, i32 1
  store i64 %5, ptr %7, align 8
  %8 = getelementptr inbounds { ptr, i64 }, ptr %_9, i32 0, i32 0
  %_2.0 = load ptr, ptr %8, align 8, !noundef !4
  %9 = getelementptr inbounds { ptr, i64 }, ptr %_9, i32 0, i32 1
  %_2.1 = load i64, ptr %9, align 8, !noundef !4
  ret void
}

; <alloc::raw_vec::RawVec<T,A> as core::ops::drop::Drop>::drop
; Function Attrs: nonlazybind uwtable
define void @"_ZN77_$LT$alloc..raw_vec..RawVec$LT$T$C$A$GT$$u20$as$u20$core..ops..drop..Drop$GT$4drop17hec8ad348e6c6cc12E"(ptr align 8 %self) unnamed_addr #1 {
start:
  %_2 = alloca %"core::option::Option<(core::ptr::non_null::NonNull<u8>, core::alloc::layout::Layout)>", align 8
; call alloc::raw_vec::RawVec<T,A>::current_memory
  call void @"_ZN5alloc7raw_vec19RawVec$LT$T$C$A$GT$14current_memory17hed0268053cefe6f7E"(ptr sret(%"core::option::Option<(core::ptr::non_null::NonNull<u8>, core::alloc::layout::Layout)>") align 8 %_2, ptr align 8 %self)
  %0 = getelementptr inbounds %"core::option::Option<(core::ptr::non_null::NonNull<u8>, core::alloc::layout::Layout)>", ptr %_2, i32 0, i32 1
  %1 = load i64, ptr %0, align 8, !range !7, !noundef !4
  %2 = icmp eq i64 %1, 0
  %_4 = select i1 %2, i64 0, i64 1
  %3 = icmp eq i64 %_4, 1
  br i1 %3, label %bb2, label %bb4

bb2:                                              ; preds = %start
  %ptr = load ptr, ptr %_2, align 8, !nonnull !4, !noundef !4
  %4 = getelementptr inbounds { ptr, { i64, i64 } }, ptr %_2, i32 0, i32 1
  %5 = getelementptr inbounds { i64, i64 }, ptr %4, i32 0, i32 0
  %layout.0 = load i64, ptr %5, align 8, !range !6, !noundef !4
  %6 = getelementptr inbounds { i64, i64 }, ptr %4, i32 0, i32 1
  %layout.1 = load i64, ptr %6, align 8, !noundef !4
  %_7 = getelementptr i8, ptr %self, i64 16
; call <alloc::alloc::Global as core::alloc::Allocator>::deallocate
  call void @"_ZN63_$LT$alloc..alloc..Global$u20$as$u20$core..alloc..Allocator$GT$10deallocate17h73cb97a91aa474baE"(ptr align 1 %_7, ptr %ptr, i64 %layout.0, i64 %layout.1)
  br label %bb4

bb4:                                              ; preds = %bb2, %start
  ret void
}

; probe1::probe
; Function Attrs: nonlazybind uwtable
define void @_ZN6probe15probe17h2fc149f31f20cb85E() unnamed_addr #1 {
start:
  %_0.i = alloca { ptr, ptr }, align 8
  %_7 = alloca [1 x { ptr, ptr }], align 8
  %_3 = alloca %"core::fmt::Arguments<'_>", align 8
  %res = alloca %"alloc::string::String", align 8
  %_1 = alloca %"alloc::string::String", align 8
  store ptr @alloc_53973d2fe29b4adba8bb7390b5678745, ptr %_0.i, align 8
  %0 = getelementptr inbounds { ptr, ptr }, ptr %_0.i, i32 0, i32 1
  store ptr @"_ZN4core3fmt3num3imp55_$LT$impl$u20$core..fmt..LowerExp$u20$for$u20$isize$GT$3fmt17h0a8586f58a2c8084E", ptr %0, align 8
  %1 = load ptr, ptr %_0.i, align 8, !nonnull !4, !align !5, !noundef !4
  %2 = getelementptr inbounds { ptr, ptr }, ptr %_0.i, i32 0, i32 1
  %3 = load ptr, ptr %2, align 8, !nonnull !4, !noundef !4
  %4 = insertvalue { ptr, ptr } poison, ptr %1, 0
  %5 = insertvalue { ptr, ptr } %4, ptr %3, 1
  %_8.0 = extractvalue { ptr, ptr } %5, 0
  %_8.1 = extractvalue { ptr, ptr } %5, 1
  %6 = getelementptr inbounds [1 x { ptr, ptr }], ptr %_7, i64 0, i64 0
  %7 = getelementptr inbounds { ptr, ptr }, ptr %6, i32 0, i32 0
  store ptr %_8.0, ptr %7, align 8
  %8 = getelementptr inbounds { ptr, ptr }, ptr %6, i32 0, i32 1
  store ptr %_8.1, ptr %8, align 8
; call core::fmt::Arguments::new_v1
  call void @_ZN4core3fmt9Arguments6new_v117hcb77c7334b09c897E(ptr sret(%"core::fmt::Arguments<'_>") align 8 %_3, ptr align 8 @alloc_ffa3cdb3ae88e54a1cc225f31dd07672, i64 1, ptr align 8 %_7, i64 1)
; call alloc::fmt::format
  call void @_ZN5alloc3fmt6format17hde97e0b1aee510c6E(ptr sret(%"alloc::string::String") align 8 %res, ptr align 8 %_3)
  call void @llvm.memcpy.p0.p0.i64(ptr align 8 %_1, ptr align 8 %res, i64 24, i1 false)
; call core::ptr::drop_in_place<alloc::string::String>
  call void @"_ZN4core3ptr42drop_in_place$LT$alloc..string..String$GT$17h2a2332ac5eb62d91E"(ptr align 8 %_1)
  ret void
}

; core::fmt::num::imp::<impl core::fmt::LowerExp for isize>::fmt
; Function Attrs: nonlazybind uwtable
declare zeroext i1 @"_ZN4core3fmt3num3imp55_$LT$impl$u20$core..fmt..LowerExp$u20$for$u20$isize$GT$3fmt17h0a8586f58a2c8084E"(ptr align 8, ptr align 8) unnamed_addr #1

; core::panicking::panic_fmt
; Function Attrs: cold noinline noreturn nonlazybind uwtable
declare void @_ZN4core9panicking9panic_fmt17hbf0e066aabfa482cE(ptr align 8, ptr align 8) unnamed_addr #2

; Function Attrs: nonlazybind uwtable
declare i32 @rust_eh_personality(i32, i32, i64, ptr, ptr) unnamed_addr #1

; core::panicking::panic_in_cleanup
; Function Attrs: cold noinline noreturn nounwind nonlazybind uwtable
declare void @_ZN4core9panicking16panic_in_cleanup17hd44bb2114362504eE() unnamed_addr #3

; Function Attrs: nocallback nofree nosync nounwind willreturn memory(inaccessiblemem: readwrite)
declare void @llvm.assume(i1 noundef) #4

; Function Attrs: nocallback nofree nosync nounwind willreturn memory(none)
declare i1 @llvm.expect.i1(i1, i1) #5

; core::panicking::panic
; Function Attrs: cold noinline noreturn nonlazybind uwtable
declare void @_ZN4core9panicking5panic17h195fc2a96298d4c3E(ptr align 1, i64, ptr align 8) unnamed_addr #2

; Function Attrs: nocallback nofree nounwind willreturn memory(argmem: readwrite)
declare void @llvm.memcpy.p0.p0.i64(ptr noalias nocapture writeonly, ptr noalias nocapture readonly, i64, i1 immarg) #6

; alloc::fmt::format::format_inner
; Function Attrs: nonlazybind uwtable
declare void @_ZN5alloc3fmt6format12format_inner17hf6fc27e89bdb9586E(ptr sret(%"alloc::string::String") align 8, ptr align 8) unnamed_addr #1

; Function Attrs: nounwind nonlazybind allockind("alloc,uninitialized,aligned") allocsize(0) uwtable
declare noalias ptr @__rust_alloc(i64, i64 allocalign) unnamed_addr #7

; Function Attrs: nounwind nonlazybind allockind("alloc,zeroed,aligned") allocsize(0) uwtable
declare noalias ptr @__rust_alloc_zeroed(i64, i64 allocalign) unnamed_addr #8

; alloc::alloc::handle_alloc_error
; Function Attrs: cold noreturn nonlazybind uwtable
declare void @_ZN5alloc5alloc18handle_alloc_error17h7d585e222775d2b3E(i64, i64) unnamed_addr #9

; alloc::raw_vec::capacity_overflow
; Function Attrs: noinline noreturn nonlazybind uwtable
declare void @_ZN5alloc7raw_vec17capacity_overflow17ha5d418fbc836d11cE() unnamed_addr #10

; Function Attrs: nounwind nonlazybind allockind("free") uwtable
declare void @__rust_dealloc(ptr allocptr, i64, i64) unnamed_addr #11

attributes #0 = { inlinehint nonlazybind uwtable "probe-stack"="inline-asm" "target-cpu"="x86-64" }
attributes #1 = { nonlazybind uwtable "probe-stack"="inline-asm" "target-cpu"="x86-64" }
attributes #2 = { cold noinline noreturn nonlazybind uwtable "probe-stack"="inline-asm" "target-cpu"="x86-64" }
attributes #3 = { cold noinline noreturn nounwind nonlazybind uwtable "probe-stack"="inline-asm" "target-cpu"="x86-64" }
attributes #4 = { nocallback nofree nosync nounwind willreturn memory(inaccessiblemem: readwrite) }
attributes #5 = { nocallback nofree nosync nounwind willreturn memory(none) }
attributes #6 = { nocallback nofree nounwind willreturn memory(argmem: readwrite) }
attributes #7 = { nounwind nonlazybind allockind("alloc,uninitialized,aligned") allocsize(0) uwtable "alloc-family"="__rust_alloc" "probe-stack"="inline-asm" "target-cpu"="x86-64" }
attributes #8 = { nounwind nonlazybind allockind("alloc,zeroed,aligned") allocsize(0) uwtable "alloc-family"="__rust_alloc" "probe-stack"="inline-asm" "target-cpu"="x86-64" }
attributes #9 = { cold noreturn nonlazybind uwtable "probe-stack"="inline-asm" "target-cpu"="x86-64" }
attributes #10 = { noinline noreturn nonlazybind uwtable "probe-stack"="inline-asm" "target-cpu"="x86-64" }
attributes #11 = { nounwind nonlazybind allockind("free") uwtable "alloc-family"="__rust_alloc" "probe-stack"="inline-asm" "target-cpu"="x86-64" }
attributes #12 = { noreturn }
attributes #13 = { cold }
attributes #14 = { cold noreturn nounwind }
attributes #15 = { nounwind }

!llvm.module.flags = !{!0, !1}
!llvm.ident = !{!2}

!0 = !{i32 8, !"PIC Level", i32 2}
!1 = !{i32 2, !"RtLibUseGOT", i32 1}
!2 = !{!"rustc version 1.75.0 (82e1608df 2023-12-21)"}
!3 = !{i64 8}
!4 = !{}
!5 = !{i64 1}
!6 = !{i64 1, i64 -9223372036854775807}
!7 = !{i64 0, i64 -9223372036854775807}
!8 = !{i8 0, i8 2}
