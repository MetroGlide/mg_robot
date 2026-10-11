import { NavigationActivity } from "../../types";
import { TONE_TEXT } from "../ui/Pill";
import { activityTone, formatActivity } from "./activityText";

/** 走行の「いまの活動」を 1 行で出す。届いていないときは何も出さない */
export default function ActivityLine({
  activity,
  className = "",
}: {
  activity: NavigationActivity | null;
  className?: string;
}) {
  if (activity === null) return null;
  return (
    <p className={`${TONE_TEXT[activityTone(activity)]} ${className}`}>
      {formatActivity(activity)}
    </p>
  );
}
