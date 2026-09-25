import { act, render } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import JoystickPad from "./JoystickPad";

function createClient(status: FoxgloveClientHandle["status"] = "connected") {
  const publish = vi.fn();
  const release = vi.fn();
  const advertise = vi.fn(() => release);
  const client: FoxgloveClientHandle = {
    status,
    subscribe: vi.fn(() => () => {}),
    callService: vi.fn(),
    publish,
    advertise,
    getLastMessageAt: () => null,
  };
  return { client, publish, advertise, release };
}

function velocities(publish: ReturnType<typeof vi.fn>): number[] {
  return publish.mock.calls.map((call) => call[2].linear.x);
}

function setHidden(hidden: boolean) {
  Object.defineProperty(document, "hidden", {
    configurable: true,
    get: () => hidden,
  });
  document.dispatchEvent(new Event("visibilitychange"));
}

// jsdom には PointerEvent がなく座標を持てないため、MouseEvent で代用する
function firePointer(svg: Element, type: string, x = 0, y = 0) {
  act(() => {
    svg.dispatchEvent(
      new MouseEvent(type, { clientX: x, clientY: y, bubbles: true }),
    );
  });
}

function startDrag(container: HTMLElement) {
  const svg = container.querySelector("svg")!;
  svg.setPointerCapture = vi.fn();
  svg.getBoundingClientRect = () =>
    ({ left: 0, top: 0, width: 136, height: 136 }) as DOMRect;
  firePointer(svg, "pointerdown", 68, 68);
  // 中心から真上に 26px (半径 52px の半分) 動かす → 前進 0.5 * maxLinear
  firePointer(svg, "pointermove", 68, 42);
  return svg;
}

beforeEach(() => {
  vi.useFakeTimers();
  setHidden(false);
});

afterEach(() => {
  vi.useRealTimers();
});

describe("JoystickPad", () => {
  it("advertises cmd_vel ahead of time and releases it on unmount", () => {
    const { client, advertise, release } = createClient();

    const { unmount } = render(<JoystickPad client={client} />);
    expect(advertise).toHaveBeenCalledWith(
      "/cmd_vel",
      "geometry_msgs/msg/Twist",
    );

    unmount();
    expect(release).toHaveBeenCalled();
  });

  it("publishes the velocity periodically while dragging", () => {
    const { client, publish } = createClient();
    const { container } = render(<JoystickPad client={client} />);

    startDrag(container);
    act(() => {
      vi.advanceTimersByTime(300);
    });

    expect(velocities(publish)).toEqual([0.25, 0.25, 0.25]);
  });

  it("sends zero repeatedly when the pointer is released", () => {
    const { client, publish } = createClient();
    const { container } = render(<JoystickPad client={client} />);
    const svg = startDrag(container);
    act(() => {
      vi.advanceTimersByTime(100);
    });
    publish.mockClear();

    firePointer(svg, "pointerup");
    act(() => {
      vi.advanceTimersByTime(500);
    });

    expect(velocities(publish)).toEqual([0, 0, 0]);
  });

  it("sends zero and stops publishing when unmounted while driving", () => {
    const { client, publish } = createClient();
    const { container, unmount } = render(<JoystickPad client={client} />);
    startDrag(container);
    act(() => {
      vi.advanceTimersByTime(100);
    });
    publish.mockClear();

    unmount();
    vi.advanceTimersByTime(1000);

    expect(velocities(publish)).toEqual([0, 0, 0]);
  });

  it("sends zero when the tab becomes hidden", () => {
    const { client, publish } = createClient();
    const { container } = render(<JoystickPad client={client} />);
    startDrag(container);
    act(() => {
      vi.advanceTimersByTime(100);
    });
    publish.mockClear();

    act(() => setHidden(true));
    act(() => {
      vi.advanceTimersByTime(1000);
    });

    expect(velocities(publish)).toEqual([0, 0, 0]);
  });

  it("sends zero when the window loses focus", () => {
    const { client, publish } = createClient();
    const { container } = render(<JoystickPad client={client} />);
    startDrag(container);
    publish.mockClear();

    act(() => {
      window.dispatchEvent(new Event("blur"));
      vi.advanceTimersByTime(1000);
    });

    expect(velocities(publish)).toEqual([0, 0, 0]);
  });

  it("does not publish anything when idle", () => {
    const { client, publish } = createClient();
    render(<JoystickPad client={client} />);

    vi.advanceTimersByTime(1000);
    window.dispatchEvent(new Event("blur"));

    expect(publish).not.toHaveBeenCalled();
  });

  it("stops publishing when the connection drops during a drag", () => {
    const { client, publish } = createClient();
    const { container, rerender } = render(<JoystickPad client={client} />);
    startDrag(container);
    act(() => {
      vi.advanceTimersByTime(100);
    });
    publish.mockClear();

    rerender(<JoystickPad client={{ ...client, status: "disconnected" }} />);
    rerender(<JoystickPad client={{ ...client, status: "connected" }} />);
    act(() => {
      vi.advanceTimersByTime(1000);
    });

    expect(publish).not.toHaveBeenCalled();
  });
});
