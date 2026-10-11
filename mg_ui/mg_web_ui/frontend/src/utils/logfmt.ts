/**
 * イベントログ(`evt=<domain>.<event> key=value key="空白を含む値" ...`)の読み取り。
 * 書式は doc/monitoring.md。Python 版(mg_utils.event_log.parse_event)と同じ規則で、
 * 同じテストデータで確かめている。
 */

export interface ParsedEvent {
  /** `<domain>.<event>` */
  name: string;
  domain: string;
  fields: Record<string, string>;
}

const EVENT_KEY = "evt";
const PAIR = /([A-Za-z_][A-Za-z0-9_]*)=("(?:[^"\\]|\\.)*"|\S*)/g;
const UNESCAPES: Record<string, string> = {
  "\\": "\\",
  '"': '"',
  n: "\n",
  r: "\r",
  t: "\t",
};

function unquote(body: string): string {
  let out = "";
  for (let i = 0; i < body.length; i++) {
    if (body[i] === "\\" && i + 1 < body.length) {
      out += UNESCAPES[body[i + 1]] ?? body[i + 1];
      i++;
    } else {
      out += body[i];
    }
  }
  return out;
}

/** 本文がイベント(先頭が `evt=`)なら読み取る。イベントでなければ null。値は文字列のまま。 */
export function parseEvent(message: string): ParsedEvent | null {
  if (!message.startsWith(`${EVENT_KEY}=`)) return null;
  const fields: Record<string, string> = {};
  for (const match of message.matchAll(PAIR)) {
    const [, key, raw] = match;
    fields[key] = raw.startsWith('"') ? unquote(raw.slice(1, -1)) : raw;
  }
  const name = fields[EVENT_KEY];
  delete fields[EVENT_KEY];
  return { name, domain: name.split(".", 1)[0], fields };
}
