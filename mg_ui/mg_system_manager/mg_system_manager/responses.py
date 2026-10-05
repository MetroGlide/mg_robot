import logging

logger = logging.getLogger(__name__)

_MAX_OK_LOG_LENGTH = 200


def result(ok: bool, msg: str) -> dict:
    """操作結果を {success, message} 形式で返し、ログにも残す。

    失敗時はトレースバックの全文を調べられるよう、メッセージを切り詰めない。
    """
    level = logging.INFO if ok else logging.WARNING
    log_msg = msg
    if ok and len(msg) > _MAX_OK_LOG_LENGTH:
        log_msg = msg[:_MAX_OK_LOG_LENGTH] + "..."
    logger.log(level, "response success=%s message=%s", ok, log_msg)
    return {"success": ok, "message": msg}
