use criterion::{black_box, criterion_group, criterion_main, Criterion};
use mrpt_core::clock::Clock;

fn bench_clock_now(c: &mut Criterion) {
    c.bench_function("Clock::now", |b| {
        b.iter(|| {
            black_box(Clock::now());
        });
    });
}

fn bench_clock_now_double(c: &mut Criterion) {
    c.bench_function("Clock::now_double", |b| {
        b.iter(|| {
            black_box(Clock::now_double());
        });
    });
}

fn bench_clock_conversions(c: &mut Criterion) {
    c.bench_function("Clock::from_double", |b| {
        b.iter(|| {
            black_box(Clock::from_double(black_box(1234567.890)));
        });
    });

    c.bench_function("Clock::to_double", |b| {
        b.iter(|| {
            black_box(Clock::to_double(black_box(123456789)));
        });
    });
}

criterion_group!(benches, bench_clock_now, bench_clock_now_double, bench_clock_conversions);
criterion_main!(benches);
