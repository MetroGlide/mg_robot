import { describe, expect, it } from "vitest";
import type { NavPvt, NavSatFix } from "../types";
import { fromNavPvt, fromNavSatFix, toUtm } from "./gnssReading";

const pvt = (overrides: Partial<NavPvt> = {}): NavPvt => ({
  fix_type: 3,
  flags: 1,
  num_sv: 20,
  lon: 1397671250,
  lat: 356812360,
  height: 40100,
  h_msl: 3200,
  h_acc: 14,
  v_acc: 21,
  p_dop: 112,
  ...overrides,
});

const fix = (overrides: Partial<NavSatFix> = {}): NavSatFix =>
  ({
    status: { status: 0, service: 1 },
    latitude: 35.681236,
    longitude: 139.767125,
    altitude: 40.1,
    position_covariance: [4, 0, 0, 0, 1, 0, 0, 0, 9],
    position_covariance_type: 2,
    ...overrides,
  }) as NavSatFix;

describe("fromNavPvt", () => {
  it("搬送波位相の解の状態で RTK の Fixed と Float を分ける", () => {
    expect(fromNavPvt(pvt({ flags: 1 | (2 << 6) }))).toMatchObject({
      label: "RTK Fixed",
      tone: "ok",
    });
    expect(fromNavPvt(pvt({ flags: 1 | (1 << 6) }))).toMatchObject({
      label: "RTK Float",
      tone: "warn",
    });
  });

  it("RTK でない測位は fix_type の表示名にする", () => {
    expect(fromNavPvt(pvt())).toMatchObject({ label: "3D Fix", tone: "warn" });
    expect(fromNavPvt(pvt({ fix_type: 0 }))).toMatchObject({
      label: "No Fix",
      tone: "error",
    });
  });

  it("単位を m・度・DOP に直す", () => {
    const r = fromNavPvt(pvt());
    expect(r.lat).toBeCloseTo(35.681236, 6);
    expect(r.lon).toBeCloseTo(139.767125, 6);
    expect(r.heightEllipsoid).toBeCloseTo(40.1);
    expect(r.heightMsl).toBeCloseTo(3.2);
    expect(r.hAcc).toBeCloseTo(0.014);
    expect(r.vAcc).toBeCloseTo(0.021);
    expect(r.pDop).toBeCloseTo(1.12);
    expect(r.numSv).toBe(20);
  });

  it("測位できていないときは、位置を持たない", () => {
    const r = fromNavPvt(pvt({ fix_type: 0, lat: 0, lon: 0 }));
    expect(r.lat).toBeNull();
    expect(r.lon).toBeNull();
  });
});

describe("fromNavSatFix", () => {
  it("status を表示名と色にする", () => {
    expect(fromNavSatFix(fix({ status: { status: 2, service: 1 } }))).toMatchObject({
      label: "RTK",
      tone: "ok",
    });
    expect(fromNavSatFix(fix({ status: { status: -1, service: 1 } }))).toMatchObject({
      label: "No Fix",
      tone: "error",
    });
    expect(fromNavSatFix(fix()).label).toBe("GNSS");
  });

  it("共分散から精度を求め、DOP と衛星数は持たない", () => {
    const r = fromNavSatFix(fix());
    expect(r.hAcc).toBeCloseTo(2);
    expect(r.vAcc).toBeCloseTo(3);
    expect(r.pDop).toBeNull();
    expect(r.numSv).toBeNull();
  });

  it("共分散が不明なら精度を持たない", () => {
    const r = fromNavSatFix(fix({ position_covariance_type: 0 }));
    expect(r.hAcc).toBeNull();
    expect(r.vAcc).toBeNull();
  });
});

describe("toUtm", () => {
  it("東京は 54N のゾーンになる", () => {
    const utm = toUtm(35.681236, 139.767125);
    expect(utm.zone).toBe("54N");
    expect(utm.easting).toBeGreaterThan(380000);
    expect(utm.easting).toBeLessThan(395000);
    expect(utm.northing).toBeGreaterThan(3940000);
    expect(utm.northing).toBeLessThan(3955000);
  });

  it("南半球は S を付ける", () => {
    expect(toUtm(-33.87, 151.21).zone).toBe("56S");
  });
});
