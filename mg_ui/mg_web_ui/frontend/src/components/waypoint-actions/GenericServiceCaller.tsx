import { useEffect, useState } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useActionRunner } from "../../hooks/useActionRunner";
import {
  buildMessageTemplate,
  describeServiceResponse,
  parseJsonObject,
} from "../../utils/waypointActions";
import ActionButton from "../ui/ActionButton";
import ActionResultText from "./ActionResultText";

const LIST_ID = "waypoint-actions-service-list";

interface GenericServiceCallerProps {
  client: FoxgloveClientHandle;
}

/** サービス名とリクエストの JSON を指定して、任意のサービスを呼ぶ。 */
export default function GenericServiceCaller({
  client,
}: GenericServiceCallerProps) {
  const [service, setService] = useState("");
  const [requestJson, setRequestJson] = useState("{}");
  const [services, setServices] = useState<string[]>([]);
  const [inputError, setInputError] = useState<string | null>(null);
  const { busy, result, run } = useActionRunner();

  // 接続が変わると client も変わる。サービスの公開は少し遅れるので、入力欄に触れたときにも取り直す
  useEffect(() => setServices(client.listServices()), [client]);

  const requestSchema = client.getServiceRequestSchema(service.trim());

  const handleInsertTemplate = () => {
    setInputError(null);
    if (!requestSchema) {
      setInputError("Service not found: the request type is unknown");
      return;
    }
    try {
      setRequestJson(
        JSON.stringify(buildMessageTemplate(requestSchema.schema), null, 2),
      );
    } catch (e) {
      setInputError(e instanceof Error ? e.message : String(e));
    }
  };

  const handleCall = () => {
    setInputError(null);
    const name = service.trim();
    if (name === "") {
      setInputError("Service name is required");
      return;
    }
    const parsed = parseJsonObject(requestJson);
    if (!parsed.ok) {
      setInputError(`Request JSON: ${parsed.error}`);
      return;
    }
    void run(async () =>
      describeServiceResponse(await client.callService(name, parsed.value)),
    );
  };

  return (
    <div className="space-y-2">
      <label className="block text-xs text-muted">
        Service
        <input
          list={LIST_ID}
          value={service}
          onChange={(e) => setService(e.target.value)}
          onFocus={() => setServices(client.listServices())}
          placeholder="/node/service"
          className="mt-1 block w-full rounded bg-surface-sunken px-2 py-1 text-sm text-content"
        />
        <datalist id={LIST_ID}>
          {services.map((s) => (
            <option key={s} value={s} />
          ))}
        </datalist>
      </label>
      {requestSchema && (
        <p className="text-xs text-muted">
          Request type: {requestSchema.schemaName}
        </p>
      )}
      <label className="block text-xs text-muted">
        Request (JSON)
        <textarea
          value={requestJson}
          onChange={(e) => setRequestJson(e.target.value)}
          rows={4}
          spellCheck={false}
          className="mt-1 block w-full rounded bg-surface-sunken px-2 py-1 font-mono text-xs text-content"
        />
      </label>
      <div className="flex gap-2">
        <ActionButton
          label="Call"
          variant="blue"
          disabled={busy}
          onClick={handleCall}
        />
        <ActionButton
          label="Insert template"
          variant="gray"
          disabled={busy}
          onClick={handleInsertTemplate}
        />
      </div>
      {inputError && <p className="text-xs text-error">{inputError}</p>}
      <ActionResultText result={result} />
    </div>
  );
}
