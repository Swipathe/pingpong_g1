export class EditorApiError extends Error {
  constructor(message, status, code) {
    super(message);
    this.name = "EditorApiError";
    this.status = status;
    this.code = code;
  }
}

export function createEditorApi(sessionToken, fetchImpl = fetch) {
  async function requestJson(url, {method = "GET", body} = {}) {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 15000);
    try {
      const response = await fetchImpl(url, {
        method,
        cache: "no-store",
        signal: controller.signal,
        headers: {
          "X-Hitter-Editor-Token": sessionToken,
          ...(body === undefined ? {} : {"Content-Type": "application/json"}),
        },
        ...(body === undefined ? {} : {body: JSON.stringify(body)}),
      });
      let payload;
      try {
        payload = await response.json();
      } catch {
        throw new EditorApiError(
          "服务器返回无效 JSON",
          response.status,
          "invalid_response",
        );
      }
      if (!response.ok) {
        throw new EditorApiError(
          payload.error || "服务器请求失败",
          response.status,
          payload.code || "request_failed",
        );
      }
      return payload;
    } catch (error) {
      if (error.name === "AbortError") {
        throw new EditorApiError("请求超时，请检查 SSH 隧道", 0, "timeout");
      }
      if (error instanceof EditorApiError) {
        throw error;
      }
      throw new EditorApiError("无法连接姿态编辑服务", 0, "network");
    } finally {
      clearTimeout(timeout);
    }
  }

  async function requestArrayBuffer(url) {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 15000);
    try {
      const response = await fetchImpl(url, {
        cache: "no-store",
        signal: controller.signal,
        headers: {"X-Hitter-Editor-Token": sessionToken},
      });
      if (!response.ok) {
        let payload = {};
        try {
          payload = await response.json();
        } catch {
          payload = {};
        }
        throw new EditorApiError(
          payload.error || "网格请求失败",
          response.status,
          payload.code || "mesh_failed",
        );
      }
      return await response.arrayBuffer();
    } catch (error) {
      if (error.name === "AbortError") {
        throw new EditorApiError("网格请求超时", 0, "timeout");
      }
      if (error instanceof EditorApiError) {
        throw error;
      }
      throw new EditorApiError("无法读取机器人网格", 0, "network");
    } finally {
      clearTimeout(timeout);
    }
  }

  return {
    health: () => requestJson("/api/health"),
    model: () => requestJson("/api/model"),
    poses: () => requestJson("/api/poses"),
    pose: (poseId) => {
      if (!/^hitter_ready_arm_pose_\d{8}_\d{6}(?:_\d{3})?$/.test(poseId)) {
        throw new EditorApiError("非法姿态 ID", 0, "invalid_pose_id");
      }
      return requestJson(`/api/poses/${encodeURIComponent(poseId)}`);
    },
    validate: (payload) => requestJson("/api/validate", {
      method: "POST",
      body: payload,
    }),
    save: (validationId, confirmSoftLimit) => requestJson("/api/save", {
      method: "POST",
      body: {
        validation_id: validationId,
        confirm_soft_limit: confirmSoftLimit,
      },
    }),
    mesh: (meshSetSha256, meshId) => {
      const sha256 = /^[0-9a-f]{64}$/;
      if (!sha256.test(meshSetSha256) || !sha256.test(meshId)) {
        throw new EditorApiError("非法网格标识", 0, "invalid_mesh_id");
      }
      return requestArrayBuffer(`/assets/${meshSetSha256}/${meshId}`);
    },
  };
}
