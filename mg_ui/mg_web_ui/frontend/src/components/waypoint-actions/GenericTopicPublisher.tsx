import { useEffect, useState } from "react";
import {
  FoxgloveClientHandle,
  GraphTopic,
} from "../../hooks/useFoxgloveClient";
import { useActionRunner } from "../../hooks/useActionRunner";
import {
  buildMessageTemplate,
  parseJsonObject,
} from "../../utils/waypointActions";
import ActionButton from "../ui/ActionButton";
import ActionResultText from "./ActionResultText";

const LIST_ID = "waypoint-actions-topic-list";

interface GenericTopicPublisherProps {
  client: FoxgloveClientHandle;
}

/** トピック名・型・メッセージの JSON を指定して、任意のトピックに 1 回だけ publish する。 */
export default function GenericTopicPublisher({
  client,
}: GenericTopicPublisherProps) {
  const [topic, setTopic] = useState("");
  const [schemaName, setSchemaName] = useState("");
  const [dataJson, setDataJson] = useState("{}");
  const [topics, setTopics] = useState<GraphTopic[]>([]);
  const [inputError, setInputError] = useState<string | null>(null);
  const { busy, result, run } = useActionRunner();

  // 接続が変わると client も変わる。トピックの公開は少し遅れるので、入力欄に触れたときにも取り直す
  useEffect(() => setTopics(client.listTopics()), [client]);

  const handleTopicChange = (value: string) => {
    setTopic(value);
    // 公開されているトピックなら、型を自動で入れる
    const known = client.listTopics().find((t) => t.topic === value.trim());
    if (known) setSchemaName(known.schemaName);
  };

  const schemaKnown = client.getMessageSchema(schemaName.trim()) !== undefined;

  const handleInsertTemplate = () => {
    setInputError(null);
    const schema = client.getMessageSchema(schemaName.trim());
    if (!schema) {
      setInputError("The schema of this type is unknown");
      return;
    }
    try {
      setDataJson(JSON.stringify(buildMessageTemplate(schema.schema), null, 2));
    } catch (e) {
      setInputError(e instanceof Error ? e.message : String(e));
    }
  };

  const handlePublish = () => {
    setInputError(null);
    const name = topic.trim();
    const type = schemaName.trim();
    if (name === "" || type === "") {
      setInputError("Topic name and type are required");
      return;
    }
    const parsed = parseJsonObject(dataJson);
    if (!parsed.ok) {
      setInputError(`Message JSON: ${parsed.error}`);
      return;
    }
    void run(async () => {
      await client.publishOnce(name, type, parsed.value);
      return { ok: true, text: "Published once" };
    });
  };

  return (
    <div className="space-y-2">
      <label className="block text-xs text-muted">
        Topic
        <input
          list={LIST_ID}
          value={topic}
          onChange={(e) => handleTopicChange(e.target.value)}
          onFocus={() => setTopics(client.listTopics())}
          placeholder="/topic"
          className="mt-1 block w-full rounded bg-surface-sunken px-2 py-1 text-sm text-content"
        />
        <datalist id={LIST_ID}>
          {topics.map((t) => (
            <option key={t.topic} value={t.topic} label={t.schemaName} />
          ))}
        </datalist>
      </label>
      <label className="block text-xs text-muted">
        Type
        <input
          value={schemaName}
          onChange={(e) => setSchemaName(e.target.value)}
          placeholder="std_msgs/msg/Bool"
          className="mt-1 block w-full rounded bg-surface-sunken px-2 py-1 text-sm text-content"
        />
      </label>
      {schemaName.trim() !== "" && !schemaKnown && (
        <p className="text-xs text-warn">
          No topic with this type is published, so the message is sent as JSON.
        </p>
      )}
      <label className="block text-xs text-muted">
        Message (JSON)
        <textarea
          value={dataJson}
          onChange={(e) => setDataJson(e.target.value)}
          rows={4}
          spellCheck={false}
          className="mt-1 block w-full rounded bg-surface-sunken px-2 py-1 font-mono text-xs text-content"
        />
      </label>
      <div className="flex gap-2">
        <ActionButton
          label="Publish once"
          variant="blue"
          disabled={busy}
          onClick={handlePublish}
        />
        <ActionButton
          label="Insert template"
          variant="gray"
          disabled={busy}
          onClick={handleInsertTemplate}
        />
      </div>
      <p className="text-xs text-muted">
        Messages are volatile: a subscriber that starts later does not receive
        them.
      </p>
      {inputError && <p className="text-xs text-error">{inputError}</p>}
      <ActionResultText result={result} />
    </div>
  );
}
