import React from 'react';

interface NestedSquareLogoProps {
  size?: number;
  className?: string;
}

export const NestedSquareLogo: React.FC<NestedSquareLogoProps> = ({ size = 20, className }) => {
  return (
    <svg
      width={size}
      height={size}
      viewBox="0 0 24 24"
      fill="none"
      xmlns="http://www.w3.org/2000/svg"
      className={className}
      aria-label="AVR Dyadic Resolution Logo"
    >
      {/* L3: 40 cm outer boundary */}
      <rect x="2" y="2" width="20" height="20" rx="1.5" stroke="#A18AC8" strokeWidth="1.2" />
      {/* L2: 20 cm subdivision */}
      <rect x="5.5" y="5.5" width="13" height="13" rx="1" stroke="#D6B45F" strokeWidth="1.2" />
      {/* L1: 10 cm subdivision */}
      <rect x="9" y="9" width="6" height="6" rx="0.5" stroke="#70B8E8" strokeWidth="1.2" />
      {/* L0: 5 cm core focal cell */}
      <rect x="10.5" y="10.5" width="3" height="3" fill="#62D3C6" />
    </svg>
  );
};
