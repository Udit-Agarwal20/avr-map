import { useEffect, useState } from 'react';
import { FrameMetadata, frameRuntime } from './PerceptionFrameRuntime';

/**
 * @brief Hook allowing React components to subscribe strictly to lightweight frame metadata.
 * Ensures zero point array or cell tree dependencies in React re-render cycles.
 */
export function useFrameMetadata(): FrameMetadata {
  const [metadata, setMetadata] = useState<FrameMetadata>(() => frameRuntime.getMetadata());

  useEffect(() => {
    return frameRuntime.subscribeMetadata((newMeta) => {
      setMetadata(newMeta);
    });
  }, []);

  return metadata;
}
