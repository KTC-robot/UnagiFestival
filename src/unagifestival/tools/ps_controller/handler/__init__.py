"""
Controller入力処理moduleの公開APIを定義する。

外部moduleから利用するHandlerのみを公開し、内部実装を隠蔽する。
"""

from unagifestival.tools.ps_controller.handler.handler import Handler

__all__ = ["Handler"]
